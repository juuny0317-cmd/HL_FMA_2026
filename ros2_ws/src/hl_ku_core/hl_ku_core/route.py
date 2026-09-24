"""Route file model and nearest/lookahead queries."""

from __future__ import annotations

import bisect
import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List


ALLOWED_MISSIONS = frozenset(
    {
        "NORMAL",
        "HILL",
        "S_OBSTACLE",
        "TRAFFIC",
        "PERP_PARK",
        "DUMMY",
        "PARALLEL_PARK",
        "END_LANE",
        "FINISH",
    }
)

FSM_ZONE_TO_MISSION = {
    "ROUTE": "NORMAL",
    "BEND": "NORMAL",
    "HILL": "HILL",
    "S_CURVE": "S_OBSTACLE",
    "TRAFFIC": "TRAFFIC",
    "T_PARK": "PERP_PARK",
    "DUMMY": "DUMMY",
    "PARALLEL_PARK": "PARALLEL_PARK",
    "END_LANE": "END_LANE",
    "FINISH": "FINISH",
}


@dataclass(frozen=True)
class Waypoint:
    index: int
    s_m: float
    x_m: float
    y_m: float
    target_speed_mps: float
    mission: str
    direction: int
    fsm_zone: str = ""
    fsm_event_ids: str = ""
    fsm_actions: str = ""
    fsm_conditions: str = ""
    fsm_hold_sec: float = 0.0
    direction_source: str = ""


class Route:
    def __init__(self, waypoints: Iterable[Waypoint]) -> None:
        self.waypoints: List[Waypoint] = list(waypoints)
        if len(self.waypoints) < 2:
            raise ValueError("A route needs at least two waypoints")
        previous_s = -math.inf
        for waypoint in self.waypoints:
            if not all(
                math.isfinite(value)
                for value in (
                    waypoint.s_m,
                    waypoint.x_m,
                    waypoint.y_m,
                    waypoint.target_speed_mps,
                )
            ):
                raise ValueError(f"route waypoint {waypoint.index} is non-finite")
            if waypoint.s_m <= previous_s:
                raise ValueError("route s_m values must be strictly increasing")
            if waypoint.target_speed_mps < 0.0:
                raise ValueError("route target speeds must be non-negative")
            if waypoint.mission not in ALLOWED_MISSIONS:
                raise ValueError(f"unknown mission tag: {waypoint.mission}")
            if waypoint.direction not in (-1, 1):
                raise ValueError("route direction must be -1 or 1")
            if not math.isfinite(waypoint.fsm_hold_sec) or waypoint.fsm_hold_sec < 0.0:
                raise ValueError("route fsm_hold_sec must be finite and non-negative")
            previous_s = waypoint.s_m
        self._s_values = [waypoint.s_m for waypoint in self.waypoints]
        self._event_indices = [
            index
            for index, waypoint in enumerate(self.waypoints)
            if waypoint.fsm_event_ids
        ]
        self.has_fsm_metadata = any(
            waypoint.fsm_zone or waypoint.fsm_event_ids
            for waypoint in self.waypoints
        )

    @classmethod
    def load_csv(
        cls, path: str | Path, variant_id: str | None = None
    ) -> "Route":
        points: List[Waypoint] = []
        accumulated = 0.0
        previous = None
        with Path(path).open(newline="", encoding="utf-8") as stream:
            reader = csv.DictReader(stream)
            has_variant_column = "variant_id" in (reader.fieldnames or ())
            selected_variant = variant_id.strip() if variant_id else ""
            if has_variant_column and not selected_variant:
                raise ValueError(
                    "route bundle requires an explicit variant_id"
                )
            if selected_variant and not has_variant_column:
                raise ValueError("variant_id was supplied for a single-route CSV")
            rows = (
                row for row in reader
                if not has_variant_column or row.get("variant_id", "").strip() == selected_variant
            )
            for index, row in enumerate(rows):
                x_m = float(row["x_m"])
                y_m = float(row["y_m"])
                if previous is not None:
                    accumulated += math.hypot(x_m - previous[0], y_m - previous[1])
                s_text = row.get("s_m", "").strip()
                s_m = float(s_text) if s_text else accumulated
                direction = int(row.get("direction", "1") or "1")
                if direction not in (-1, 1):
                    raise ValueError(f"direction must be -1 or 1 at row {index + 2}")
                fsm_zone = (row.get("fsm_zone", "") or "").strip().upper()
                if fsm_zone and fsm_zone not in FSM_ZONE_TO_MISSION:
                    raise ValueError(f"unknown fsm_zone at row {index + 2}: {fsm_zone}")
                mission = (
                    FSM_ZONE_TO_MISSION[fsm_zone]
                    if fsm_zone
                    else (row.get("mission", "NORMAL") or "NORMAL").strip().upper()
                )
                points.append(
                    Waypoint(
                        index=index,
                        s_m=s_m,
                        x_m=x_m,
                        y_m=y_m,
                        target_speed_mps=float(row["target_speed_mps"]),
                        mission=mission,
                        direction=direction,
                        fsm_zone=fsm_zone,
                        fsm_event_ids=(row.get("fsm_event_ids", "") or "").strip(),
                        fsm_actions=(row.get("fsm_actions", "") or "").strip(),
                        fsm_conditions=(row.get("fsm_conditions", "") or "").strip(),
                        fsm_hold_sec=float(row.get("fsm_hold_sec", "") or 0.0),
                        direction_source=(row.get("direction_source", "") or "").strip(),
                    )
                )
                previous = (x_m, y_m)
        if not points:
            suffix = f" for variant_id={selected_variant!r}" if selected_variant else ""
            raise ValueError(f"route CSV has no waypoints{suffix}")
        return cls(points)

    def nearest_index(self, x_m: float, y_m: float, hint: int = 0, window: int = 80) -> int:
        start = max(0, hint - max(5, window // 4))
        stop = min(len(self.waypoints), max(start + 1, hint + window))
        return min(
            range(start, stop),
            key=lambda index: (
                (self.waypoints[index].x_m - x_m) ** 2
                + (self.waypoints[index].y_m - y_m) ** 2
            ),
        )

    def nearest_index_between(
        self,
        x_m: float,
        y_m: float,
        start_index: int,
        end_index: int,
    ) -> int:
        """Return the closest point inside one authored motion phase."""

        start = max(0, min(int(start_index), len(self.waypoints) - 1))
        end = max(start, min(int(end_index), len(self.waypoints) - 1))
        return min(
            range(start, end + 1),
            key=lambda index: (
                (self.waypoints[index].x_m - x_m) ** 2
                + (self.waypoints[index].y_m - y_m) ** 2
            ),
        )

    def lookahead_index(self, start_index: int, lookahead_m: float) -> int:
        target_s = self.waypoints[start_index].s_m + max(0.0, lookahead_m)
        for index in range(start_index, len(self.waypoints)):
            if self.waypoints[index].s_m >= target_s:
                return index
        return len(self.waypoints) - 1

    def nearest_segment_index(
        self,
        x_m: float,
        y_m: float,
        hint: int = 0,
        window: int = 80,
    ) -> int:
        """Return the first waypoint index of the closest local segment."""
        return self.nearest_projection(x_m, y_m, hint, window)[0]

    def nearest_projection(
        self,
        x_m: float,
        y_m: float,
        hint: int = 0,
        window: int = 80,
    ) -> tuple[int, float, float, float]:
        """Return segment index, route s and XY of the closest projection."""
        start = max(0, hint - max(5, window // 4))
        stop = min(len(self.waypoints) - 1, max(start + 1, hint + window))

        def project(index: int) -> tuple[float, float, float, float]:
            first = self.waypoints[index]
            second = self.waypoints[index + 1]
            dx = second.x_m - first.x_m
            dy = second.y_m - first.y_m
            length_sq = dx * dx + dy * dy
            if length_sq <= 1.0e-12:
                ratio = 0.0
            else:
                ratio = max(
                    0.0,
                    min(
                        1.0,
                        ((x_m - first.x_m) * dx + (y_m - first.y_m) * dy)
                        / length_sq,
                    ),
                )
            closest_x = first.x_m + ratio * dx
            closest_y = first.y_m + ratio * dy
            distance_sq = (x_m - closest_x) ** 2 + (y_m - closest_y) ** 2
            route_s = first.s_m + ratio * (second.s_m - first.s_m)
            return distance_sq, route_s, closest_x, closest_y

        index, result = min(
            ((index, project(index)) for index in range(start, stop)),
            key=lambda item: item[1][0],
        )
        return index, result[1], result[2], result[3]

    def nearest_projection_between(
        self,
        x_m: float,
        y_m: float,
        start_index: int,
        end_index: int,
    ) -> tuple[int, float, float, float]:
        """Project only onto segments in one forward or reverse phase."""

        start = max(0, min(int(start_index), len(self.waypoints) - 1))
        end = max(start, min(int(end_index), len(self.waypoints) - 1))
        if start == end:
            point = self.waypoints[start]
            return start, point.s_m, point.x_m, point.y_m
        segment_end = end - 1

        def project(index: int) -> tuple[float, float, float, float]:
            first = self.waypoints[index]
            second = self.waypoints[index + 1]
            dx = second.x_m - first.x_m
            dy = second.y_m - first.y_m
            length_sq = dx * dx + dy * dy
            ratio = 0.0 if length_sq <= 1.0e-12 else max(
                0.0,
                min(
                    1.0,
                    ((x_m - first.x_m) * dx + (y_m - first.y_m) * dy)
                    / length_sq,
                ),
            )
            closest_x = first.x_m + ratio * dx
            closest_y = first.y_m + ratio * dy
            distance_sq = (x_m - closest_x) ** 2 + (y_m - closest_y) ** 2
            route_s = first.s_m + ratio * (second.s_m - first.s_m)
            return distance_sq, route_s, closest_x, closest_y

        index, result = min(
            (
                (index, project(index))
                for index in range(start, segment_end + 1)
            ),
            key=lambda item: item[1][0],
        )
        return index, result[1], result[2], result[3]

    def heading_at_s(self, s_m: float, baseline_m: float = 1.0) -> float:
        """Estimate a noise-resistant route tangent over an arc-length baseline."""
        first_s = self.waypoints[0].s_m
        final_s = self.waypoints[-1].s_m
        baseline = min(max(1.0e-3, baseline_m), final_s - first_s)
        center = max(first_s, min(s_m, final_s))
        start = max(first_s, center - 0.5 * baseline)
        end = min(final_s, start + baseline)
        start = max(first_s, end - baseline)
        first = self.position_at_s(start)
        second = self.position_at_s(end)
        return math.atan2(second[1] - first[1], second[0] - first[0])

    def position_at_s(self, s_m: float) -> tuple[float, float]:
        """Linearly interpolate a point along the measured route polyline."""
        if s_m <= self.waypoints[0].s_m:
            first = self.waypoints[0]
            return first.x_m, first.y_m
        if s_m >= self.waypoints[-1].s_m:
            last = self.waypoints[-1]
            return last.x_m, last.y_m
        upper = bisect.bisect_right(self._s_values, s_m)
        lower = upper - 1
        first = self.waypoints[lower]
        second = self.waypoints[upper]
        ratio = (s_m - first.s_m) / (second.s_m - first.s_m)
        return (
            first.x_m + ratio * (second.x_m - first.x_m),
            first.y_m + ratio * (second.y_m - first.y_m),
        )

    def curvature_ahead(
        self,
        start_index: int,
        preview_m: float,
        segment_count: int = 3,
    ) -> float:
        """Estimate the largest absolute heading change per metre ahead.

        Equal arc-length samples make the classification independent of route
        waypoint spacing.  Multiple segments also retain an S-bend that a
        single start-to-end heading comparison could cancel out.
        """
        start = self.waypoints[max(0, min(start_index, len(self.waypoints) - 1))].s_m
        end = min(self.waypoints[-1].s_m, start + max(0.0, preview_m))
        span = end - start
        count = max(2, int(segment_count))
        if span <= 1.0e-3:
            return 0.0
        step = span / count
        points = [self.position_at_s(start + step * index) for index in range(count + 1)]
        headings = [
            math.atan2(second[1] - first[1], second[0] - first[0])
            for first, second in zip(points, points[1:])
        ]
        changes = [
            abs(math.atan2(math.sin(second - first), math.cos(second - first))) / step
            for first, second in zip(headings, headings[1:])
        ]
        return max(changes, default=0.0)

    def metadata_waypoint(
        self,
        nearest_index: int,
        x_m: float,
        y_m: float,
        finish_tolerance_m: float,
    ) -> Waypoint:
        """Delay FINISH speed/tag until the vehicle reaches the final point."""
        nearest = self.waypoints[nearest_index]
        if nearest.mission != "FINISH":
            return nearest
        final = self.waypoints[-1]
        distance = math.hypot(final.x_m - x_m, final.y_m - y_m)
        if distance <= max(0.0, finish_tolerance_m):
            return final
        for waypoint in reversed(self.waypoints[:nearest_index]):
            if waypoint.mission != "FINISH":
                return waypoint
        return nearest

    def active_event_waypoint(self, nearest_index: int) -> Waypoint:
        """Return the latest authored event at or before the current route point."""
        stop = max(0, min(int(nearest_index), len(self.waypoints) - 1))
        position = bisect.bisect_right(self._event_indices, stop) - 1
        if position >= 0:
            return self.waypoints[self._event_indices[position]]
        return self.waypoints[0]
