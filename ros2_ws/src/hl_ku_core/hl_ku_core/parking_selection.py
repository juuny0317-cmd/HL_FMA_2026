"""LiDAR-based T/parallel parking branch selection without ROS dependencies."""

from __future__ import annotations

import math
import re
from dataclasses import dataclass
from enum import Enum
from pathlib import Path

from .mando_route_slice import slice_and_place
from .route import Route, Waypoint


_VARIANT_PATTERN = re.compile(r"^p([12])_t([12])_f([12])$", re.IGNORECASE)


class BayState(str, Enum):
    UNKNOWN = "UNKNOWN"
    CLEAR = "CLEAR"
    BLOCKED = "BLOCKED"


@dataclass(frozen=True)
class VehiclePose2D:
    x_m: float
    y_m: float
    yaw_rad: float


@dataclass(frozen=True)
class ObstacleDisc:
    x_m: float
    y_m: float
    radius_m: float


@dataclass(frozen=True)
class BayGeometry:
    mission: str
    option: int
    path: tuple[tuple[float, float], ...]


@dataclass(frozen=True)
class ParkingSelectionSettings:
    assessment_tail_length_m: float = 4.0
    corridor_half_width_m: float = 0.55
    ambiguous_assignment_m: float = 0.10
    minimum_visible_fraction: float = 0.35
    minimum_visible_points: int = 4
    clear_scans_required: int = 3
    blocked_scans_required: int = 5
    lidar_minimum_range_m: float = 0.50
    # Parking-space selection only needs the nearby bay.  Keeping this shorter
    # than the general obstacle-mapper range prevents distant walls from being
    # counted as cones without reducing the S-course avoidance range.
    lidar_maximum_range_m: float = 5.0
    lidar_front_half_angle_rad: float = math.radians(70.0)
    lidar_x_m: float = 1.25
    lidar_y_m: float = 0.0
    decision_lead_m: float = 0.75


def parse_variant_id(variant_id: str) -> tuple[int, int, int]:
    match = _VARIANT_PATTERN.fullmatch(variant_id.strip())
    if match is None:
        raise ValueError(f"invalid parking route variant: {variant_id!r}")
    return tuple(int(value) for value in match.groups())


def variant_id(parallel: int, perpendicular: int, final: int) -> str:
    if parallel not in (1, 2) or perpendicular not in (1, 2) or final not in (1, 2):
        raise ValueError("parking route options must be 1 or 2")
    return f"p{parallel}_t{perpendicular}_f{final}"


def _event_waypoint(route: Route, event_id: str) -> Waypoint:
    for waypoint in route.waypoints:
        if event_id in waypoint.fsm_event_ids.split("|"):
            return waypoint
    raise ValueError(f"route has no event {event_id}")


def _longest_reverse_run(route: Route, mission: str) -> tuple[Waypoint, ...]:
    runs: list[list[Waypoint]] = []
    current: list[Waypoint] = []
    for waypoint in route.waypoints:
        if waypoint.mission == mission and waypoint.direction < 0:
            current.append(waypoint)
        elif current:
            runs.append(current)
            current = []
    if current:
        runs.append(current)
    if not runs:
        raise ValueError(f"route has no reverse {mission} run")
    return tuple(max(runs, key=lambda run: run[-1].s_m - run[0].s_m))


def _tail_path(route: Route, mission: str, length_m: float) -> tuple[tuple[float, float], ...]:
    reverse = _longest_reverse_run(route, mission)
    start_s = reverse[-1].s_m - max(0.1, length_m)
    selected = [point for point in reverse if point.s_m >= start_s]
    if len(selected) < 2:
        selected = list(reverse[-2:])
    # Dense source routes can contain thousands of points.  About 25 cm between
    # occupancy samples is enough for visibility and distance checks.
    output: list[tuple[float, float]] = []
    for point in selected:
        xy = (point.x_m, point.y_m)
        if not output or math.hypot(xy[0] - output[-1][0], xy[1] - output[-1][1]) >= 0.20:
            output.append(xy)
    final = (selected[-1].x_m, selected[-1].y_m)
    if output[-1] != final:
        output.append(final)
    return tuple(output)


@dataclass(frozen=True)
class ParkingRouteFamily:
    routes: dict[str, Route]
    initial_variant_id: str
    settings: ParkingSelectionSettings

    @classmethod
    def from_placed_route(
        cls,
        active_route: Route,
        source_bundle: str | Path,
        initial_variant_id: str,
        source_start_s_m: float,
        source_length_m: float,
        speed_mps: float,
        settings: ParkingSelectionSettings | None = None,
    ) -> "ParkingRouteFamily":
        parallel, perpendicular, final = parse_variant_id(initial_variant_id)
        del parallel, perpendicular, final
        if source_length_m <= 0.0:
            raise ValueError("source_length_m must be positive for parking selection")
        if speed_mps <= 0.0:
            raise ValueError("speed_mps must be positive for parking selection")
        first, second = active_route.waypoints[:2]
        heading_deg = math.degrees(
            math.atan2(second.y_m - first.y_m, second.x_m - first.x_m)
        )
        reference_source = Route.load_csv(source_bundle, initial_variant_id.lower())
        routes: dict[str, Route] = {}
        for p_option in (1, 2):
            for t_option in (1, 2):
                for final_option in (1, 2):
                    route_id = variant_id(p_option, t_option, final_option)
                    source = Route.load_csv(source_bundle, route_id)
                    routes[route_id] = slice_and_place(
                        source,
                        source_start_s_m,
                        source_length_m,
                        first.x_m,
                        first.y_m,
                        heading_deg,
                        speed_mps,
                        preserve_source_profile=True,
                        placement_source=reference_source,
                    )
        return cls(routes, initial_variant_id.lower(), settings or ParkingSelectionSettings())

    @property
    def final_option(self) -> int:
        return parse_variant_id(self.initial_variant_id)[2]

    def route(self, parallel: int, perpendicular: int) -> Route:
        return self.routes[variant_id(parallel, perpendicular, self.final_option)]

    def bay(self, mission: str, option: int, perpendicular: int = 1) -> BayGeometry:
        if mission == "PERP_PARK":
            route = self.route(1, option)
        elif mission == "PARALLEL_PARK":
            route = self.route(option, perpendicular)
        else:
            raise ValueError(f"unsupported parking mission: {mission}")
        return BayGeometry(
            mission,
            option,
            _tail_path(route, mission, self.settings.assessment_tail_length_m),
        )

    def decision_s_m(self, mission: str, perpendicular: int = 1) -> float:
        if mission == "PERP_PARK":
            return _event_waypoint(
                self.route(1, 1), "T1_FORWARD_TO_REVERSE"
            ).s_m
        if mission != "PARALLEL_PARK":
            raise ValueError(f"unsupported parking mission: {mission}")

        p1_route = self.route(1, perpendicular)
        p2_route = self.route(2, perpendicular)
        p2_turn = _event_waypoint(p2_route, "PARALLEL_2_FORWARD_TO_REVERSE")
        parallel_indices = [
            index for index, point in enumerate(p1_route.waypoints)
            if point.mission == "PARALLEL_PARK" and point.direction > 0
        ]
        if len(parallel_indices) < 2:
            raise ValueError("P1 route has no forward parallel-parking approach")
        start = parallel_indices[0]
        # Only project on the first forward approach, before P1 reverses.
        end = start
        while (
            end + 1 < len(p1_route.waypoints)
            and p1_route.waypoints[end + 1].mission == "PARALLEL_PARK"
            and p1_route.waypoints[end + 1].direction > 0
        ):
            end += 1
        _, projected_s, _, _ = p1_route.nearest_projection_between(
            p2_turn.x_m, p2_turn.y_m, start, end
        )
        return projected_s


def _distance_to_path(x_m: float, y_m: float, path: tuple[tuple[float, float], ...]) -> float:
    best = math.inf
    for first, second in zip(path, path[1:]):
        dx = second[0] - first[0]
        dy = second[1] - first[1]
        length_sq = dx * dx + dy * dy
        ratio = 0.0 if length_sq <= 1.0e-12 else max(
            0.0,
            min(1.0, ((x_m - first[0]) * dx + (y_m - first[1]) * dy) / length_sq),
        )
        nearest_x = first[0] + ratio * dx
        nearest_y = first[1] + ratio * dy
        best = min(best, math.hypot(x_m - nearest_x, y_m - nearest_y))
    return best


def choose_option(first: BayState, second: BayState) -> int:
    """Choose one bay using the user's fallback table; never return stop/none."""
    if first == BayState.CLEAR:
        return 1
    if first == BayState.BLOCKED:
        return 2 if second != BayState.BLOCKED else 1
    # T1/P1 was not observed.  Trust the one observed state for option 2.
    if second == BayState.CLEAR:
        return 2
    return 1


class ParkingBranchSelector:
    """Accumulate bay observations and latch one branch at each decision point."""

    def __init__(self, family: ParkingRouteFamily) -> None:
        self.family = family
        self.parallel_option = 1
        self.perpendicular_option = 1
        self.active_mission = ""
        self._clear_scans = {1: 0, 2: 0}
        self._blocked_scans = {1: 0, 2: 0}
        self._blocked = {1: False, 2: False}
        self._locked = {"PERP_PARK": False, "PARALLEL_PARK": False}

    def set_active_mission(self, mission: str) -> None:
        mission = mission.upper()
        if mission == self.active_mission:
            return
        self.active_mission = mission if mission in ("PERP_PARK", "PARALLEL_PARK") else ""
        if not self.active_mission:
            return
        self._clear_scans = {1: 0, 2: 0}
        self._blocked_scans = {1: 0, 2: 0}
        self._blocked = {1: False, 2: False}
        self._locked[self.active_mission] = False

    def restore_variant(self, route_id: str, active_mission: str = "") -> None:
        """Restore the route branch selected before a full-stack restart."""

        parallel, perpendicular, _final = parse_variant_id(route_id)
        self.parallel_option = parallel
        self.perpendicular_option = perpendicular
        mission = active_mission.strip().upper()
        self.active_mission = (
            mission if mission in ("PERP_PARK", "PARALLEL_PARK") else ""
        )
        self._clear_scans = {1: 0, 2: 0}
        self._blocked_scans = {1: 0, 2: 0}
        self._blocked = {1: False, 2: False}
        if self.active_mission:
            self._locked[self.active_mission] = True

    def state(self, option: int) -> BayState:
        if self._blocked[option]:
            return BayState.BLOCKED
        if self._clear_scans[option] >= self.family.settings.clear_scans_required:
            return BayState.CLEAR
        return BayState.UNKNOWN

    def _visible(self, pose: VehiclePose2D, bay: BayGeometry) -> bool:
        settings = self.family.settings
        cosine = math.cos(pose.yaw_rad)
        sine = math.sin(pose.yaw_rad)
        lidar_x = pose.x_m + cosine * settings.lidar_x_m - sine * settings.lidar_y_m
        lidar_y = pose.y_m + sine * settings.lidar_x_m + cosine * settings.lidar_y_m
        visible = 0
        for x_m, y_m in bay.path:
            dx = x_m - lidar_x
            dy = y_m - lidar_y
            distance = math.hypot(dx, dy)
            bearing = math.atan2(
                math.sin(math.atan2(dy, dx) - pose.yaw_rad),
                math.cos(math.atan2(dy, dx) - pose.yaw_rad),
            )
            if (
                settings.lidar_minimum_range_m <= distance <= settings.lidar_maximum_range_m
                and abs(bearing) <= settings.lidar_front_half_angle_rad
            ):
                visible += 1
        required = max(
            settings.minimum_visible_points,
            math.ceil(len(bay.path) * settings.minimum_visible_fraction),
        )
        return visible >= required

    def observe(self, pose: VehiclePose2D, obstacles: tuple[ObstacleDisc, ...]) -> None:
        mission = self.active_mission
        if mission not in ("PERP_PARK", "PARALLEL_PARK") or self._locked[mission]:
            return
        bays = {
            option: self.family.bay(mission, option, self.perpendicular_option)
            for option in (1, 2)
        }
        visible = {option: self._visible(pose, bay) for option, bay in bays.items()}
        hit = {1: False, 2: False}
        settings = self.family.settings
        cosine = math.cos(pose.yaw_rad)
        sine = math.sin(pose.yaw_rad)
        lidar_x = pose.x_m + cosine * settings.lidar_x_m - sine * settings.lidar_y_m
        lidar_y = pose.y_m + sine * settings.lidar_x_m + cosine * settings.lidar_y_m
        for obstacle in obstacles:
            if not all(
                math.isfinite(value)
                for value in (obstacle.x_m, obstacle.y_m, obstacle.radius_m)
            ) or obstacle.radius_m < 0.0:
                continue
            dx = obstacle.x_m - lidar_x
            dy = obstacle.y_m - lidar_y
            distance = math.hypot(dx, dy)
            bearing = math.atan2(
                math.sin(math.atan2(dy, dx) - pose.yaw_rad),
                math.cos(math.atan2(dy, dx) - pose.yaw_rad),
            )
            if not (
                settings.lidar_minimum_range_m
                <= distance
                <= settings.lidar_maximum_range_m
                and abs(bearing) <= settings.lidar_front_half_angle_rad
            ):
                continue
            distances = {
                option: _distance_to_path(obstacle.x_m, obstacle.y_m, bay.path)
                for option, bay in bays.items()
            }
            nearest = min(distances, key=distances.get)
            other = 2 if nearest == 1 else 1
            # A cone equally close to both paths is a divider, not evidence that
            # both parking spaces are occupied.
            if abs(distances[nearest] - distances[other]) < settings.ambiguous_assignment_m:
                continue
            if distances[nearest] - obstacle.radius_m <= settings.corridor_half_width_m:
                hit[nearest] = True

        for option in (1, 2):
            if hit[option]:
                self._clear_scans[option] = 0
                self._blocked_scans[option] += 1
                if (
                    self._blocked_scans[option]
                    >= self.family.settings.blocked_scans_required
                ):
                    self._blocked[option] = True
            elif visible[option]:
                self._blocked_scans[option] = 0
                self._clear_scans[option] += 1

    def maybe_lock(self, route_s_m: float) -> bool:
        mission = self.active_mission
        if mission not in ("PERP_PARK", "PARALLEL_PARK") or self._locked[mission]:
            return False
        decision_s = self.family.decision_s_m(mission, self.perpendicular_option)
        # Keep accumulating observations throughout the common approach and
        # commit only immediately before the first mandatory reverse point.
        # Every route candidate now uses one shared rigid placement, so the
        # approach remains aligned and a late T1/T2 or P1/P2 switch is smooth.
        if route_s_m < decision_s - self.family.settings.decision_lead_m:
            return False
        selected = choose_option(self.state(1), self.state(2))
        if mission == "PERP_PARK":
            self.perpendicular_option = selected
        else:
            self.parallel_option = selected
        self._locked[mission] = True
        return True

    def selected_variant_id(self) -> str:
        return variant_id(
            self.parallel_option,
            self.perpendicular_option,
            self.family.final_option,
        )

    def status(self) -> dict[str, object]:
        mission = self.active_mission
        return {
            "mission": mission,
            "locked": bool(mission and self._locked[mission]),
            "option_1": self.state(1).value if mission else BayState.UNKNOWN.value,
            "option_2": self.state(2).value if mission else BayState.UNKNOWN.value,
            "selected_t": self.perpendicular_option,
            "selected_p": self.parallel_option,
            "selected_variant_id": self.selected_variant_id(),
        }
