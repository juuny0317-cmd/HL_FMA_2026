"""Apply regulation-oriented mission decisions to the route command."""

from __future__ import annotations

import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
from nav_msgs.msg import Path as PathMessage
from rclpy.node import Node
from std_msgs.msg import Float32, String, UInt8
from std_srvs.srv import Trigger

from hl_ku_interfaces.msg import (
    DriveCommand,
    MissionStatus,
    ObstacleArray,
    PedestrianFrame,
    PerceptionState,
)

from .geometry import clamp
from .detour_gate import select_detour_command
from .detour_following import DetourFollowResult, DetourFollowSettings, follow_detour
from .mission import MissionCoordinator, Observation, State
from .pedestrian import PedestrianVoteSession
from .parking_selection import (
    ObstacleDisc,
    ParkingBranchSelector,
    ParkingRouteFamily,
    ParkingSelectionSettings,
    VehiclePose2D,
)
from .path_tracker_node import quaternion_yaw
from .route import Route
from .traffic_light import (
    SIGNAL_GREEN_BIT,
    SIGNAL_LEFT_BIT,
    SIGNAL_RED_BIT,
    SIGNAL_YELLOW_BIT,
    TrafficSignalVoteSession,
    TrafficSignalVoter,
)


class MissionManagerNode(Node):
    def __init__(self) -> None:
        super().__init__("mission_manager")
        self.declare_parameter("auto_arm", False)
        self.declare_parameter("default_speed_mps", 1.0)
        self.declare_parameter("hill_stop_s_m", 0.0)
        self.declare_parameter("hill_hold_sec", 3.0)
        self.declare_parameter("hill_speed_mps", 0.7)
        self.declare_parameter("dummy_trigger_m", 4.0)
        self.declare_parameter("dummy_hold_sec", 3.2)
        self.declare_parameter("pedestrian_hold_sec", 3.5)
        self.declare_parameter("traffic_stop_trigger_m", 2.0)
        self.declare_parameter("enable_yolo_missions", False)
        self.declare_parameter("use_lidar_dummy_stop", False)
        self.declare_parameter("traffic_vote_window_frames", 1)
        self.declare_parameter("red_required_votes", 1)
        self.declare_parameter("yellow_required_votes", 1)
        self.declare_parameter("green_required_votes", 1)
        self.declare_parameter("left_required_votes", 1)
        self.declare_parameter("traffic_vote_stale_sec", 0.5)
        self.declare_parameter("pedestrian_vote_window_frames", 5)
        self.declare_parameter("pedestrian_required_votes", 3)
        self.declare_parameter("pedestrian_clear_consecutive_frames", 5)
        self.declare_parameter("pedestrian_vote_stale_sec", 0.5)
        self.declare_parameter("pedestrian_velocity_stale_sec", 0.5)
        self.declare_parameter("maximum_steering_rad", 0.48)
        self.declare_parameter("path_command_timeout_sec", 0.20)
        self.declare_parameter("enable_local_detour_steering", False)
        self.declare_parameter("detour_candidate_timeout_sec", 0.75)
        self.declare_parameter("detour_pose_timeout_sec", 0.40)
        self.declare_parameter("detour_speed_limit_mps", 0.30)
        self.declare_parameter("detour_stanley_gain", 0.65)
        self.declare_parameter("detour_stanley_softening_mps", 0.65)
        self.declare_parameter("detour_stanley_control_offset_m", 0.58)
        self.declare_parameter("detour_stanley_heading_baseline_m", 1.0)
        self.declare_parameter("detour_maximum_steering_rad", 0.30)
        self.declare_parameter("enable_parking_selection", False)
        self.declare_parameter("parking_source_route_file", "")
        self.declare_parameter("parking_source_variant_id", "")
        self.declare_parameter("parking_source_start_s_m", 0.0)
        self.declare_parameter("parking_source_length_m", 0.0)
        self.declare_parameter("parking_active_route_file", "")
        self.declare_parameter("parking_assessment_tail_length_m", 4.0)
        self.declare_parameter("parking_corridor_half_width_m", 0.55)
        self.declare_parameter("parking_ambiguous_assignment_m", 0.10)
        self.declare_parameter("parking_minimum_visible_fraction", 0.35)
        self.declare_parameter("parking_minimum_visible_points", 4)
        self.declare_parameter("parking_clear_scans_required", 3)
        self.declare_parameter("parking_blocked_scans_required", 5)
        self.declare_parameter("parking_lidar_minimum_range_m", 0.50)
        self.declare_parameter("parking_lidar_maximum_range_m", 5.0)
        self.declare_parameter("parking_lidar_front_half_angle_deg", 70.0)
        self.declare_parameter("parking_lidar_x_m", 1.25)
        self.declare_parameter("parking_lidar_y_m", 0.0)
        self.declare_parameter("parking_decision_lead_m", 0.75)
        self.declare_parameter("parking_pose_timeout_sec", 0.75)
        self._coordinator = MissionCoordinator(
            default_speed_mps=float(self.get_parameter("default_speed_mps").value),
            hill_stop_s_m=float(self.get_parameter("hill_stop_s_m").value),
            hill_hold_sec=float(self.get_parameter("hill_hold_sec").value),
            hill_speed_mps=float(self.get_parameter("hill_speed_mps").value),
            dummy_trigger_m=float(self.get_parameter("dummy_trigger_m").value),
            dummy_hold_sec=float(self.get_parameter("dummy_hold_sec").value),
            pedestrian_hold_sec=float(
                self.get_parameter("pedestrian_hold_sec").value
            ),
            traffic_stop_trigger_m=float(self.get_parameter("traffic_stop_trigger_m").value),
        )
        if bool(self.get_parameter("auto_arm").value):
            self._coordinator.arm()
        self._path_command: DriveCommand | None = None
        self._path_command_time = None
        self._perception = PerceptionState()
        self._perception.stop_line_distance_m = math.inf
        self._perception.obstacle_distance_m = math.inf
        self._route_s = 0.0
        self._zone = "NORMAL"
        self._route_event = {
            "event_id": "",
            "action": "",
            "condition": "",
            "hold_sec": 0.0,
            "has_fsm_metadata": False,
            "relocalized": False,
            "direction": 1,
            "variant_id": "",
            "current_zone": "NORMAL",
        }
        self._resume_context_applied = False
        self._speed = 0.0
        self._speed_time: float | None = None
        self._yolo_missions_enabled = bool(
            self.get_parameter("enable_yolo_missions").value
        )
        self._lidar_dummy_stop_enabled = bool(
            self.get_parameter("use_lidar_dummy_stop").value
        )
        self._traffic_vote_session = TrafficSignalVoteSession(
            TrafficSignalVoter(
                int(self.get_parameter("traffic_vote_window_frames").value),
                red_required_votes=int(
                    self.get_parameter("red_required_votes").value
                ),
                yellow_required_votes=int(
                    self.get_parameter("yellow_required_votes").value
                ),
                green_required_votes=int(
                    self.get_parameter("green_required_votes").value
                ),
                left_required_votes=int(
                    self.get_parameter("left_required_votes").value
                ),
            ),
            stale_sec=float(self.get_parameter("traffic_vote_stale_sec").value),
        )
        self._pedestrian_vote_session = PedestrianVoteSession(
            window_frames=int(
                self.get_parameter("pedestrian_vote_window_frames").value
            ),
            required_votes=int(
                self.get_parameter("pedestrian_required_votes").value
            ),
            clear_consecutive_frames=int(
                self.get_parameter("pedestrian_clear_consecutive_frames").value
            ),
            stale_sec=float(
                self.get_parameter("pedestrian_vote_stale_sec").value
            ),
        )
        self._detour_path: tuple[tuple[float, float], ...] | None = None
        self._detour_path_at: float | None = None
        self._detour_status = ""
        self._detour_status_at: float | None = None
        self._detour_pose: PoseWithCovarianceStamped | None = None
        self._detour_pose_at: float | None = None
        self._detour_engaged = False
        self._parking_selector = self._build_parking_selector()
        self._command_pub = self.create_publisher(
            DriveCommand, "/mission/command", 10
        )
        self._status_pub = self.create_publisher(MissionStatus, "/mission/status", 10)
        self._parking_pub = self.create_publisher(
            String, "/mission/parking_selection", 10
        )
        self.create_subscription(
            DriveCommand, "/planning/path_command", self._on_path_command, 10
        )
        self.create_subscription(PerceptionState, "/perception/state", self._on_perception, 10)
        self.create_subscription(
            PathMessage,
            "/planning/live/local_detour_candidate",
            self._on_detour_path,
            10,
        )
        self.create_subscription(
            String,
            "/planning/live/local_detour_status",
            self._on_detour_status,
            10,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/localization/gnss_pose",
            self._on_detour_pose,
            20,
        )
        self.create_subscription(
            ObstacleArray,
            "/planning/obstacle_discs_map",
            self._on_parking_obstacles,
            10,
        )
        self.create_subscription(Float32, "/planning/route_s", self._on_s, 10)
        self.create_subscription(String, "/planning/route_zone", self._on_zone, 10)
        self.create_subscription(String, "/planning/route_event", self._on_route_event, 10)
        self.create_subscription(
            UInt8,
            "/perception/traffic_signal_mask",
            self._on_traffic_signal_mask,
            10,
        )
        self.create_subscription(
            PedestrianFrame,
            "/perception/pedestrian_frame",
            self._on_pedestrian_frame,
            10,
        )
        self.create_subscription(
            TwistStamped, "/localization/vehicle_velocity", self._on_velocity, 20
        )
        self.create_service(Trigger, "/mission/arm", self._arm)
        self.create_service(Trigger, "/mission/toggle_run", self._toggle_run)
        self.create_service(Trigger, "/mission/fault", self._fault)
        self.create_timer(0.05, self._update)

    def _build_parking_selector(self) -> ParkingBranchSelector | None:
        if not bool(self.get_parameter("enable_parking_selection").value):
            return None
        try:
            active_route = Route.load_csv(
                str(self.get_parameter("parking_active_route_file").value)
            )
            settings = ParkingSelectionSettings(
                assessment_tail_length_m=float(
                    self.get_parameter("parking_assessment_tail_length_m").value
                ),
                corridor_half_width_m=float(
                    self.get_parameter("parking_corridor_half_width_m").value
                ),
                ambiguous_assignment_m=float(
                    self.get_parameter("parking_ambiguous_assignment_m").value
                ),
                minimum_visible_fraction=float(
                    self.get_parameter("parking_minimum_visible_fraction").value
                ),
                minimum_visible_points=int(
                    self.get_parameter("parking_minimum_visible_points").value
                ),
                clear_scans_required=int(
                    self.get_parameter("parking_clear_scans_required").value
                ),
                blocked_scans_required=int(
                    self.get_parameter("parking_blocked_scans_required").value
                ),
                lidar_minimum_range_m=float(
                    self.get_parameter("parking_lidar_minimum_range_m").value
                ),
                lidar_maximum_range_m=float(
                    self.get_parameter("parking_lidar_maximum_range_m").value
                ),
                lidar_front_half_angle_rad=math.radians(
                    float(
                        self.get_parameter("parking_lidar_front_half_angle_deg").value
                    )
                ),
                lidar_x_m=float(self.get_parameter("parking_lidar_x_m").value),
                lidar_y_m=float(self.get_parameter("parking_lidar_y_m").value),
                decision_lead_m=float(
                    self.get_parameter("parking_decision_lead_m").value
                ),
            )
            family = ParkingRouteFamily.from_placed_route(
                active_route,
                str(self.get_parameter("parking_source_route_file").value),
                str(self.get_parameter("parking_source_variant_id").value),
                float(self.get_parameter("parking_source_start_s_m").value),
                float(self.get_parameter("parking_source_length_m").value),
                max(0.01, float(self.get_parameter("default_speed_mps").value)),
                settings,
            )
        except (OSError, ValueError, KeyError) as error:
            self.get_logger().error(f"parking selector disabled: {error}")
            return None
        self.get_logger().info(
            "parking selector enabled; LiDAR decisions are limited to "
            "PERP_PARK and PARALLEL_PARK"
        )
        return ParkingBranchSelector(family)

    def _on_path_command(self, message: DriveCommand) -> None:
        self._path_command = message
        self._path_command_time = time.monotonic()

    def _on_perception(self, message: PerceptionState) -> None:
        self._perception = message

    def _on_detour_path(self, message: PathMessage) -> None:
        self._detour_path = (
            tuple((float(pose.pose.position.x), float(pose.pose.position.y)) for pose in message.poses)
            if message.header.frame_id == "map"
            else None
        )
        self._detour_path_at = time.monotonic()

    def _on_detour_status(self, message: String) -> None:
        self._detour_status = message.data
        self._detour_status_at = time.monotonic()

    def _on_detour_pose(self, message: PoseWithCovarianceStamped) -> None:
        self._detour_pose = message if message.header.frame_id == "map" else None
        self._detour_pose_at = time.monotonic()

    def _on_parking_obstacles(self, message: ObstacleArray) -> None:
        selector = self._parking_selector
        if selector is None or message.header.frame_id != "map":
            return
        selector.set_active_mission(self._zone.upper())
        pose_message = self._detour_pose
        if pose_message is None or not self._fresh(
            self._detour_pose_at,
            float(self.get_parameter("parking_pose_timeout_sec").value),
        ):
            return
        pose = pose_message.pose.pose
        yaw = quaternion_yaw(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        selector.observe(
            VehiclePose2D(float(pose.position.x), float(pose.position.y), yaw),
            tuple(
                ObstacleDisc(
                    float(obstacle.x_m),
                    float(obstacle.y_m),
                    float(obstacle.radius_m),
                )
                for obstacle in message.obstacles
            ),
        )

    @staticmethod
    def _fresh(stamp: float | None, timeout_sec: float) -> bool:
        return stamp is not None and 0.0 <= time.monotonic() - stamp <= timeout_sec

    def _detour_follow_result(self) -> DetourFollowResult:
        candidate_timeout = float(self.get_parameter("detour_candidate_timeout_sec").value)
        pose_timeout = float(self.get_parameter("detour_pose_timeout_sec").value)
        if not (
            self._perception.lidar_fresh
            and self._detour_status == "CANDIDATE"
            and self._fresh(self._detour_status_at, candidate_timeout)
            and self._fresh(self._detour_path_at, candidate_timeout)
            and self._fresh(self._detour_pose_at, pose_timeout)
            and self._detour_path is not None
            and self._detour_pose is not None
        ):
            return DetourFollowResult("UNAVAILABLE")
        pose = self._detour_pose.pose.pose
        quaternion = pose.orientation
        quaternion_norm = math.sqrt(
            quaternion.x * quaternion.x
            + quaternion.y * quaternion.y
            + quaternion.z * quaternion.z
            + quaternion.w * quaternion.w
        )
        if not math.isfinite(quaternion_norm) or not 0.90 <= quaternion_norm <= 1.10:
            return DetourFollowResult("INVALID_POSE")
        yaw = quaternion_yaw(
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w,
        )
        return follow_detour(
            self._detour_path,
            float(pose.position.x), float(pose.position.y), yaw,
            DetourFollowSettings(
                stanley_gain=float(self.get_parameter("detour_stanley_gain").value),
                stanley_softening_mps=float(
                    self.get_parameter("detour_stanley_softening_mps").value
                ),
                control_point_offset_m=float(
                    self.get_parameter("detour_stanley_control_offset_m").value
                ),
                heading_baseline_m=float(
                    self.get_parameter("detour_stanley_heading_baseline_m").value
                ),
                maximum_steering_rad=float(
                    self.get_parameter("detour_maximum_steering_rad").value
                ),
            ),
            speed_mps=self._speed,
        )

    def _on_s(self, message: Float32) -> None:
        self._route_s = float(message.data)

    def _on_zone(self, message: String) -> None:
        zone = message.data.strip().upper()
        if zone != self._zone:
            self._traffic_vote_session.set_context(
                zone, "TRAFFIC" if zone == "TRAFFIC" else "OFF"
            )
            self._pedestrian_vote_session.set_context(
                zone,
                "PEDESTRIAN" if zone == "DUMMY" else "OFF",
                "INSIDE" if zone == "DUMMY" else "OUTSIDE",
            )
        self._zone = zone

    def _on_traffic_signal_mask(self, message: UInt8) -> None:
        if not self._yolo_missions_enabled or self._zone != "TRAFFIC":
            self._traffic_vote_session.reset()
            return
        self._traffic_vote_session.update(int(message.data), time.monotonic())

    def _on_pedestrian_frame(self, message: PedestrianFrame) -> None:
        if (
            self._lidar_dummy_stop_enabled
            or not self._yolo_missions_enabled
            or self._zone != "DUMMY"
        ):
            self._pedestrian_vote_session.reset()
            return
        self._pedestrian_vote_session.update(
            bool(message.detected), time.monotonic()
        )

    def _on_route_event(self, message: String) -> None:
        try:
            value = json.loads(message.data)
            hold_sec = float(value.get("hold_sec", 0.0))
            if not math.isfinite(hold_sec) or hold_sec < 0.0:
                return
            self._route_event = {
                "event_id": str(value.get("event_id", "")),
                "action": str(value.get("action", "")),
                "condition": str(value.get("condition", "")),
                "hold_sec": hold_sec,
                "has_fsm_metadata": bool(value.get("has_fsm_metadata", False)),
                "relocalized": bool(value.get("relocalized", False)),
                "direction": int(value.get("direction", 1)),
                "variant_id": str(value.get("variant_id", "")),
                "current_zone": str(value.get("current_zone", "NORMAL")),
            }
        except (TypeError, ValueError, json.JSONDecodeError):
            self.get_logger().warning("invalid route FSM event metadata")

    def _on_velocity(self, message: TwistStamped) -> None:
        self._speed = float(message.twist.linear.x)
        self._speed_time = time.monotonic()

    def _arm(self, _request, response):
        self._traffic_vote_session.reset()
        self._pedestrian_vote_session.reset()
        self._coordinator.arm()
        response.success = self._coordinator.state != State.INIT
        response.message = "mission armed" if response.success else "arm rejected"
        return response

    def _fault(self, _request, response):
        self._traffic_vote_session.reset()
        self._pedestrian_vote_session.reset()
        self._coordinator.fault()
        response.success = True
        response.message = "mission fault latched; restart nodes to clear"
        return response

    def _toggle_run(self, _request, response):
        self._traffic_vote_session.reset()
        self._pedestrian_vote_session.reset()
        response.success, response.message = self._coordinator.toggle_run(
            time.monotonic()
        )
        return response

    def _update(self) -> None:
        now = self.get_clock().now()
        now_sec = time.monotonic()
        traffic_vote = self._traffic_vote_session.result(now_sec)
        pedestrian_vote = self._pedestrian_vote_session.result(now_sec)
        lidar_dummy_stop_active = (
            self._lidar_dummy_stop_enabled and self._zone == "DUMMY"
        )
        pedestrian_mode_active = (
            self._yolo_missions_enabled
            and self._zone == "DUMMY"
            and not lidar_dummy_stop_active
        )
        if (
            self._route_event["relocalized"]
            and not self._resume_context_applied
            and self._coordinator.state == State.READY
        ):
            self._zone = self._route_event["current_zone"].strip().upper()
            self._coordinator.resume_at_route(
                self._zone,
                self._route_event["event_id"],
            )
            variant = self._route_event["variant_id"]
            if self._parking_selector is not None and variant:
                try:
                    self._parking_selector.restore_variant(
                        variant,
                        self._zone,
                    )
                except ValueError as error:
                    self.get_logger().warning(
                        f"restart parking variant ignored: {error}"
                    )
            self._resume_context_applied = True
            self.get_logger().info(
                "mission resumed from route context: "
                f"s={self._route_s:.2f}m "
                f"zone={self._route_event['current_zone']} "
                f"event={self._route_event['event_id'] or '-'} "
                f"direction={self._route_event['direction']:+d}"
            )
        speed_age = (
            now_sec - self._speed_time if self._speed_time is not None else math.inf
        )
        previous_state = self._coordinator.state
        decision = self._coordinator.update(
            Observation(
                now_sec=now_sec,
                zone=self._zone,
                route_s_m=self._route_s,
                speed_mps=self._speed,
                stop_line_detected=self._perception.stop_line_detected,
                stop_line_distance_m=self._perception.stop_line_distance_m,
                traffic_light=self._perception.traffic_light,
                traffic_votes_active=(
                    self._yolo_missions_enabled and self._zone == "TRAFFIC"
                ),
                traffic_red_confirmed=traffic_vote.confirmed(SIGNAL_RED_BIT),
                traffic_yellow_confirmed=traffic_vote.confirmed(
                    SIGNAL_YELLOW_BIT
                ),
                traffic_green_confirmed=traffic_vote.confirmed(SIGNAL_GREEN_BIT),
                traffic_left_confirmed=traffic_vote.confirmed(SIGNAL_LEFT_BIT),
                pedestrian_mode_active=pedestrian_mode_active,
                pedestrian_stop_confirmed=(
                    pedestrian_mode_active and pedestrian_vote.stop_confirmed
                ),
                pedestrian_clear_confirmed=pedestrian_vote.clear_confirmed,
                pedestrian_speed_fresh=(
                    math.isfinite(self._speed)
                    and 0.0 <= speed_age <= float(
                        self.get_parameter("pedestrian_velocity_stale_sec").value
                    )
                ),
                lidar_dummy_stop_active=lidar_dummy_stop_active,
                lidar_fresh=bool(self._perception.lidar_fresh),
                obstacle_in_path=self._perception.obstacle_in_path,
                obstacle_distance_m=self._perception.obstacle_distance_m,
                allowed_lane=self._perception.allowed_lane,
                route_event_id=self._route_event["event_id"],
                route_event_action=self._route_event["action"],
                route_event_condition=self._route_event["condition"],
                route_event_hold_sec=self._route_event["hold_sec"],
                route_has_fsm_metadata=self._route_event["has_fsm_metadata"],
            )
        )
        if previous_state != State.DUMMY_HOLD and decision.state == State.DUMMY_HOLD:
            self._pedestrian_vote_session.begin_clear()
        parking_detail = ""
        if self._parking_selector is not None:
            parking_mission = (
                "PERP_PARK"
                if decision.state == State.PERP_PARK
                else "PARALLEL_PARK"
                if decision.state == State.PARALLEL_PARK
                else ""
            )
            self._parking_selector.set_active_mission(parking_mission)
            self._parking_selector.maybe_lock(self._route_s)
            parking_status = self._parking_selector.status()
            if parking_mission:
                prefix = "T" if parking_mission == "PERP_PARK" else "P"
                selected = parking_status[
                    "selected_t" if prefix == "T" else "selected_p"
                ]
                lock_text = "LOCKED" if parking_status["locked"] else "OBSERVING"
                parking_detail = (
                    f"{prefix}1={parking_status['option_1']} "
                    f"{prefix}2={parking_status['option_2']} -> "
                    f"{prefix}{selected} {lock_text}"
                )
            parking_message = String()
            parking_message.data = json.dumps(
                parking_status, separators=(",", ":")
            )
            self._parking_pub.publish(parking_message)
        command = DriveCommand()
        command.header.stamp = now.to_msg()
        command.header.frame_id = "base_link"
        path_age = (
            time.monotonic() - self._path_command_time
            if self._path_command_time is not None
            else math.inf
        )
        path_fresh = 0.0 <= path_age <= float(
            self.get_parameter("path_command_timeout_sec").value
        )
        path_valid = self._path_command is not None and all(
            math.isfinite(value)
            for value in (
                self._path_command.speed_mps,
                self._path_command.steering_angle_rad,
            )
        )
        if path_valid and path_fresh:
            requested = float(self._path_command.speed_mps)
            speed_limit = max(0.0, decision.speed_limit_mps)
            command.speed_mps = math.copysign(min(abs(requested), speed_limit), requested)
            command.steering_angle_rad = self._path_command.steering_angle_rad
        detour_enabled = bool(self.get_parameter("enable_local_detour_steering").value)
        gate = select_detour_command(
            enabled=detour_enabled,
            in_s_obstacle=decision.state == State.S_OBSTACLE,
            engaged=self._detour_engaged,
            speed_mps=float(command.speed_mps),
            route_steering_rad=float(command.steering_angle_rad),
            legacy_steering_offset_rad=float(
                self._perception.avoidance_steering_offset_rad
            ),
            lidar_fresh=bool(self._perception.lidar_fresh),
            obstacle_in_path=bool(self._perception.obstacle_in_path),
            obstacle_distance_m=float(self._perception.obstacle_distance_m),
            follow=(
                self._detour_follow_result()
                if detour_enabled and decision.state == State.S_OBSTACLE
                else DetourFollowResult("UNAVAILABLE")
            ),
            approach_speed_limit_mps=float(
                self.get_parameter("detour_speed_limit_mps").value
            ),
        )
        self._detour_engaged = gate.engaged
        command.speed_mps = gate.speed_mps
        command.steering_angle_rad = gate.steering_rad
        maximum = float(self.get_parameter("maximum_steering_rad").value)
        command.steering_angle_rad = clamp(command.steering_angle_rad, -maximum, maximum)
        if decision.brake or gate.brake or not path_fresh or not path_valid:
            command.speed_mps = 0.0
        self._command_pub.publish(command)
        status = MissionStatus()
        status.header = command.header
        status.state = int(decision.state)
        status.state_name = decision.state.name
        status.zone = self._zone
        status.route_s_m = self._route_s
        status.speed_limit_mps = decision.speed_limit_mps
        status.lateral_offset_m = decision.lateral_offset_m
        status.brake_required = decision.brake or gate.brake or not path_fresh or not path_valid
        status.mission_complete = decision.state == State.FINISH
        status.detail = (
            (gate.detail or parking_detail or decision.detail)
            if path_fresh and path_valid
            else "path command stale or invalid"
        )
        self._status_pub.publish(status)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()
