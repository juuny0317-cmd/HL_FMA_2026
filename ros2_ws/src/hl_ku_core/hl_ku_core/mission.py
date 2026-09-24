"""Regulation-oriented mission state machine without ROS dependencies."""

from __future__ import annotations

import dataclasses
from enum import IntEnum


class State(IntEnum):
    INIT = 0
    READY = 1
    ROUTE = 2
    HILL_APPROACH = 10
    HILL_HOLD = 11
    HILL_CLIMB = 12
    S_OBSTACLE = 20
    TRAFFIC_APPROACH = 30
    TRAFFIC_WAIT = 31
    PERP_PARK = 40
    DUMMY_ARMED = 50
    DUMMY_BRAKE = 51
    DUMMY_HOLD = 52
    PARALLEL_PARK = 60
    END_LANE = 70
    FINISH = 80
    FAULT = 255


LIGHT_UNKNOWN = 0
LIGHT_RED = 1
LIGHT_YELLOW = 2
LIGHT_GREEN = 3
LIGHT_LEFT = 4


@dataclasses.dataclass(frozen=True)
class Observation:
    now_sec: float
    zone: str
    route_s_m: float
    speed_mps: float
    stop_line_detected: bool = False
    stop_line_distance_m: float = float("inf")
    traffic_light: int = LIGHT_UNKNOWN
    traffic_votes_active: bool = False
    traffic_red_confirmed: bool = False
    traffic_yellow_confirmed: bool = False
    traffic_green_confirmed: bool = False
    traffic_left_confirmed: bool = False
    pedestrian_mode_active: bool = False
    pedestrian_stop_confirmed: bool = False
    pedestrian_clear_confirmed: bool = False
    pedestrian_speed_fresh: bool = True
    lidar_dummy_stop_active: bool = False
    lidar_fresh: bool = False
    obstacle_in_path: bool = False
    obstacle_distance_m: float = float("inf")
    allowed_lane: int = 0
    route_event_id: str = ""
    route_event_action: str = ""
    route_event_condition: str = ""
    route_event_hold_sec: float = 0.0
    route_has_fsm_metadata: bool = False


@dataclasses.dataclass(frozen=True)
class Decision:
    state: State
    speed_limit_mps: float
    brake: bool
    lateral_offset_m: float
    detail: str


class MissionCoordinator:
    def __init__(
        self,
        default_speed_mps: float = 1.0,
        hill_stop_s_m: float = 0.0,
        hill_hold_sec: float = 3.0,
        hill_speed_mps: float = 0.7,
        stopped_speed_mps: float = 0.04,
        dummy_trigger_m: float = 4.0,
        dummy_hold_sec: float = 3.2,
        pedestrian_hold_sec: float = 3.5,
        traffic_stop_trigger_m: float = 2.0,
        green_debounce_sec: float = 0.35,
        end_lane_offset_m: float = 1.5,
    ) -> None:
        self.default_speed_mps = default_speed_mps
        self.hill_stop_s_m = hill_stop_s_m
        self.hill_hold_sec = hill_hold_sec
        self.hill_speed_mps = hill_speed_mps
        self.stopped_speed_mps = stopped_speed_mps
        self.dummy_trigger_m = dummy_trigger_m
        self.dummy_hold_sec = dummy_hold_sec
        self.pedestrian_hold_sec = pedestrian_hold_sec
        self.traffic_stop_trigger_m = traffic_stop_trigger_m
        self.green_debounce_sec = green_debounce_sec
        self.end_lane_offset_m = end_lane_offset_m
        self.state = State.INIT
        self._hold_start_sec: float | None = None
        self._green_start_sec: float | None = None
        self._last_zone = ""
        self._operator_paused = False
        self._pause_start_sec: float | None = None

    @property
    def operator_paused(self) -> bool:
        return self._operator_paused

    def arm(self) -> None:
        if self.state == State.INIT:
            self.state = State.READY

    def resume_at_route(self, zone: str, route_event_id: str = "") -> None:
        """Restore the FSM that owns a globally reacquired route position."""

        normalized = zone.strip().upper()
        mapping = {
            "NORMAL": State.ROUTE,
            "HILL": State.HILL_APPROACH,
            "S_OBSTACLE": State.S_OBSTACLE,
            "TRAFFIC": State.TRAFFIC_APPROACH,
            "PERP_PARK": State.PERP_PARK,
            "DUMMY": State.DUMMY_ARMED,
            "PARALLEL_PARK": State.PARALLEL_PARK,
            "END_LANE": State.END_LANE,
            "FINISH": State.FINISH,
        }
        state = mapping.get(normalized, State.ROUTE)
        # HILL_STOP is an authored one-shot event.  Reacquiring a point beyond
        # it means the stop was already passed, so do not execute it again.
        if normalized == "HILL" and route_event_id == "HILL_STOP":
            state = State.HILL_CLIMB
        self.state = state
        self._last_zone = normalized
        self._hold_start_sec = None
        self._green_start_sec = None
        self._operator_paused = False
        self._pause_start_sec = None

    def fault(self) -> None:
        self.state = State.FAULT
        self._operator_paused = False
        self._pause_start_sec = None

    def toggle_run(self, now_sec: float) -> tuple[bool, str]:
        """Start, pause, or resume without discarding mission progress."""
        if self.state == State.FAULT:
            return False, "mission fault is latched; restart nodes to clear"
        if self.state == State.FINISH:
            return False, "course already finished"
        if self.state == State.INIT:
            self.arm()
            return True, "mission started"
        if not self._operator_paused:
            self._operator_paused = True
            self._pause_start_sec = now_sec
            return True, "mission paused"

        pause_started = (
            self._pause_start_sec
            if self._pause_start_sec is not None
            else now_sec
        )
        paused_for = max(0.0, now_sec - pause_started)
        # Pausing must not consume mandatory hold/debounce time.
        if self._hold_start_sec is not None:
            self._hold_start_sec += paused_for
        if self._green_start_sec is not None:
            self._green_start_sec += paused_for
        self._operator_paused = False
        self._pause_start_sec = None
        return True, "mission resumed"

    def _zone_entry(self, zone: str) -> None:
        if zone == self._last_zone:
            return
        self._last_zone = zone
        self._hold_start_sec = None
        self._green_start_sec = None
        mapping = {
            "NORMAL": State.ROUTE,
            "HILL": State.HILL_APPROACH,
            "S_OBSTACLE": State.S_OBSTACLE,
            "TRAFFIC": State.TRAFFIC_APPROACH,
            "PERP_PARK": State.PERP_PARK,
            "DUMMY": State.DUMMY_ARMED,
            "PARALLEL_PARK": State.PARALLEL_PARK,
            "END_LANE": State.END_LANE,
            "FINISH": State.FINISH,
        }
        self.state = mapping.get(zone, State.ROUTE)

    def update(self, observation: Observation) -> Decision:
        if self.state == State.FAULT:
            return Decision(self.state, 0.0, True, 0.0, "fault latched")
        if self.state == State.INIT:
            return Decision(self.state, 0.0, True, 0.0, "not armed")
        if self._operator_paused:
            return Decision(self.state, 0.0, True, 0.0, "operator paused")
        self._zone_entry(observation.zone.upper())

        if self.state == State.FINISH:
            return Decision(self.state, 0.0, True, 0.0, "course finished")

        if self.state == State.HILL_APPROACH:
            authored_hill_stop = (
                observation.route_event_id == "HILL_STOP"
                and observation.route_event_action == "STOP_THEN_HOLD"
                and observation.route_event_condition in ("", "ALWAYS_ONCE")
            )
            legacy_hill_stop = (
                not observation.route_has_fsm_metadata
                and observation.route_s_m >= self.hill_stop_s_m
            )
            if authored_hill_stop or legacy_hill_stop:
                # Start the powered hold as soon as the vehicle is stopped or
                # rolling backward.  Waiting for abs(speed) to fall below the
                # stopped threshold can deadlock on a slope: dynamic braking
                # may not stop the vehicle, so HILL_HOLD (and its rollback
                # assist) would otherwise never become active.
                if observation.speed_mps <= self.stopped_speed_mps:
                    self.state = State.HILL_HOLD
                    self._hold_start_sec = observation.now_sec
                return Decision(self.state, 0.0, True, 0.0, "hill stop")
            return Decision(self.state, min(0.5, self.hill_speed_mps), False, 0.0, "hill approach")

        if self.state == State.HILL_HOLD:
            if abs(observation.speed_mps) > self.stopped_speed_mps:
                self._hold_start_sec = observation.now_sec
            start = self._hold_start_sec if self._hold_start_sec is not None else observation.now_sec
            held = observation.now_sec - start
            hold_sec = (
                observation.route_event_hold_sec
                if observation.route_event_id == "HILL_STOP"
                and observation.route_event_hold_sec > 0.0
                else self.hill_hold_sec
            )
            if held >= hold_sec:
                self.state = State.HILL_CLIMB
                return Decision(self.state, self.hill_speed_mps, False, 0.0, "hill launch")
            return Decision(self.state, 0.0, True, 0.0, f"hill hold {held:.2f}s")

        if self.state == State.HILL_CLIMB:
            return Decision(self.state, self.hill_speed_mps, False, 0.0, "hill climb")

        if self.state == State.TRAFFIC_APPROACH:
            if observation.traffic_votes_active:
                if (
                    observation.traffic_red_confirmed
                    or observation.traffic_yellow_confirmed
                ):
                    self.state = State.TRAFFIC_WAIT
                    return Decision(
                        self.state,
                        0.0,
                        True,
                        0.0,
                        "YOLO traffic stop confirmed",
                    )
            else:
                must_stop = observation.traffic_light != LIGHT_GREEN
                close = (
                    observation.stop_line_detected
                    and observation.stop_line_distance_m <= self.traffic_stop_trigger_m
                )
                if must_stop and close:
                    self.state = State.TRAFFIC_WAIT
                    return Decision(self.state, 0.0, True, 0.0, "traffic stop")
            return Decision(self.state, 0.45, False, 0.0, "traffic approach")

        if self.state == State.TRAFFIC_WAIT:
            if observation.traffic_votes_active:
                if (
                    observation.traffic_left_confirmed
                    or observation.traffic_green_confirmed
                ):
                    self.state = State.ROUTE
                    return Decision(
                        self.state,
                        self.default_speed_mps,
                        False,
                        0.0,
                        "YOLO traffic go confirmed",
                    )
                return Decision(self.state, 0.0, True, 0.0, "YOLO traffic stop hold")
            if observation.traffic_light == LIGHT_GREEN:
                if self._green_start_sec is None:
                    self._green_start_sec = observation.now_sec
                if observation.now_sec - self._green_start_sec >= self.green_debounce_sec:
                    self.state = State.ROUTE
                    return Decision(self.state, self.default_speed_mps, False, 0.0, "green")
            else:
                self._green_start_sec = None
            return Decision(self.state, 0.0, True, 0.0, "red or unknown")

        if self.state == State.DUMMY_ARMED:
            if observation.lidar_dummy_stop_active:
                if not observation.lidar_fresh:
                    return Decision(
                        self.state,
                        0.0,
                        True,
                        0.0,
                        "dummy LiDAR stale",
                    )
                if (
                    observation.obstacle_in_path
                    and observation.obstacle_distance_m <= self.dummy_trigger_m
                ):
                    self.state = State.DUMMY_BRAKE
                    return Decision(
                        self.state,
                        0.0,
                        True,
                        0.0,
                        "dummy LiDAR obstacle within stop trigger",
                    )
                return Decision(
                    self.state,
                    min(1.0, self.default_speed_mps),
                    False,
                    0.0,
                    "dummy LiDAR path monitoring",
                )
            if observation.pedestrian_mode_active:
                if observation.pedestrian_stop_confirmed:
                    self.state = State.DUMMY_BRAKE
                    return Decision(
                        self.state,
                        0.0,
                        True,
                        0.0,
                        "pedestrian YOLO confirmed stop",
                    )
                return Decision(
                    self.state,
                    min(1.0, self.default_speed_mps),
                    False,
                    0.0,
                    "pedestrian monitoring",
                )
            if (
                observation.obstacle_in_path
                and observation.obstacle_distance_m <= self.dummy_trigger_m
            ):
                self.state = State.DUMMY_BRAKE
                return Decision(self.state, 0.0, True, 0.0, "dummy emergency brake")
            return Decision(self.state, min(1.0, self.default_speed_mps), False, 0.0, "dummy armed")

        if self.state == State.DUMMY_BRAKE:
            stopped = abs(observation.speed_mps) <= self.stopped_speed_mps
            if (
                observation.lidar_dummy_stop_active
                or observation.pedestrian_mode_active
            ):
                stopped = stopped and observation.pedestrian_speed_fresh
            if stopped:
                self.state = State.DUMMY_HOLD
                self._hold_start_sec = observation.now_sec
            return Decision(self.state, 0.0, True, 0.0, "waiting for full stop")

        if self.state == State.DUMMY_HOLD:
            if observation.lidar_dummy_stop_active:
                fully_stopped = (
                    abs(observation.speed_mps) <= self.stopped_speed_mps
                    and observation.pedestrian_speed_fresh
                )
                if not fully_stopped:
                    self._hold_start_sec = observation.now_sec
                    return Decision(
                        self.state,
                        0.0,
                        True,
                        0.0,
                        "dummy stop timer waiting for full stop",
                    )
                start = (
                    self._hold_start_sec
                    if self._hold_start_sec is not None
                    else observation.now_sec
                )
                held = max(0.0, observation.now_sec - start)
                if held < self.dummy_hold_sec:
                    return Decision(
                        self.state,
                        0.0,
                        True,
                        0.0,
                        f"dummy mandatory hold {held:.2f}/{self.dummy_hold_sec:.2f}s",
                    )
                if not observation.lidar_fresh:
                    return Decision(
                        self.state,
                        0.0,
                        True,
                        0.0,
                        "dummy LiDAR stale; holding",
                    )
                if observation.obstacle_in_path:
                    return Decision(
                        self.state,
                        0.0,
                        True,
                        0.0,
                        "dummy LiDAR obstacle still in path",
                    )
                # Re-arm inside the DUMMY zone so a second obstacle can stop
                # the vehicle without waiting for a zone transition.
                self.state = State.DUMMY_ARMED
                self._hold_start_sec = None
                return Decision(
                    self.state,
                    self.default_speed_mps,
                    False,
                    0.0,
                    "dummy LiDAR path clear",
                )
            if observation.pedestrian_mode_active:
                fully_stopped = (
                    abs(observation.speed_mps) <= self.stopped_speed_mps
                    and observation.pedestrian_speed_fresh
                )
                if not fully_stopped:
                    # Count the mandatory hold only while the vehicle is
                    # measurably stopped and the velocity sample is current.
                    self._hold_start_sec = observation.now_sec
                    return Decision(
                        self.state,
                        0.0,
                        True,
                        0.0,
                        "pedestrian stop timer waiting for full stop",
                    )
                start = (
                    self._hold_start_sec
                    if self._hold_start_sec is not None
                    else observation.now_sec
                )
                held = max(0.0, observation.now_sec - start)
                if (
                    held >= self.pedestrian_hold_sec
                    and observation.pedestrian_clear_confirmed
                ):
                    self.state = State.ROUTE
                    return Decision(
                        self.state,
                        self.default_speed_mps,
                        False,
                        0.0,
                        "pedestrian cleared",
                    )
                if held < self.pedestrian_hold_sec:
                    return Decision(
                        self.state,
                        0.0,
                        True,
                        0.0,
                        f"pedestrian mandatory hold {held:.2f}/{self.pedestrian_hold_sec:.2f}s",
                    )
                return Decision(
                    self.state,
                    0.0,
                    True,
                    0.0,
                    "pedestrian clear frames waiting",
                )
            if abs(observation.speed_mps) > self.stopped_speed_mps:
                self._hold_start_sec = observation.now_sec
            start = self._hold_start_sec if self._hold_start_sec is not None else observation.now_sec
            held = observation.now_sec - start
            if held >= self.dummy_hold_sec and not observation.obstacle_in_path:
                self.state = State.ROUTE
                return Decision(self.state, 0.4, False, 0.0, "dummy cleared")
            return Decision(self.state, 0.0, True, 0.0, f"dummy hold {held:.2f}s")

        if self.state == State.S_OBSTACLE:
            return Decision(self.state, 0.45, False, 0.0, "LiDAR avoidance active")

        if self.state in (State.PERP_PARK, State.PARALLEL_PARK):
            return Decision(self.state, 0.25, False, 0.0, "follow calibrated parking route")

        if self.state == State.END_LANE:
            lane = max(-1, min(1, observation.allowed_lane))
            return Decision(
                self.state,
                0.4,
                False,
                0.0,
                "follow selected F1/F2 route" if lane else "default F1 route",
            )

        self.state = State.ROUTE
        return Decision(self.state, self.default_speed_mps, False, 0.0, "route")
