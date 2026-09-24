"""End-lane sign decision and smooth F1-to-F2 route transition helpers."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Mapping, Sequence

from .route import Route, Waypoint


EXPECTED_ENDPOINT_NAMES = ("green_arrow", "red_signal")
ROUTE_1 = "ROUTE_1"
ROUTE_2 = "ROUTE_2"
UNKNOWN = "UNKNOWN"


@dataclass(frozen=True)
class EndpointDetection:
    class_name: str
    confidence: float
    x1: float
    y1: float
    x2: float
    y2: float

    @property
    def center(self) -> tuple[float, float]:
        return ((self.x1 + self.x2) * 0.5, (self.y1 + self.y2) * 0.5)

    @property
    def height(self) -> float:
        return max(0.0, self.y2 - self.y1)


@dataclass(frozen=True)
class EndpointDecision:
    route: str
    green: EndpointDetection | None
    red: EndpointDetection | None


def normalize_endpoint_names(
    names: Mapping[int, str] | Sequence[str],
) -> tuple[str, ...]:
    if isinstance(names, Mapping):
        try:
            ordered = tuple(str(names[index]).strip().lower() for index in range(len(names)))
        except (KeyError, TypeError) as error:
            raise ValueError("endpoint model class IDs must be contiguous") from error
    else:
        ordered = tuple(str(name).strip().lower() for name in names)
    if set(ordered) != set(EXPECTED_ENDPOINT_NAMES) or len(ordered) != 2:
        raise ValueError(
            f"endpoint model must contain exactly {EXPECTED_ENDPOINT_NAMES}; got {ordered}"
        )
    return ordered


def endpoint_route_candidate(
    detections: Sequence[EndpointDetection],
    frame_height: int,
) -> EndpointDecision:
    """Require both classes in one frame and compare their horizontal order."""
    greens = [item for item in detections if item.class_name == "green_arrow"]
    reds = [item for item in detections if item.class_name == "red_signal"]
    if not greens or not reds or frame_height <= 0:
        return EndpointDecision(UNKNOWN, None, None)
    green = max(greens, key=lambda item: item.confidence)
    _, green_y = green.center
    same_row = [
        red
        for red in reds
        if abs(green_y - red.center[1])
        <= max(frame_height * 0.12, max(green.height, red.height) * 1.5)
    ]
    if not same_row:
        return EndpointDecision(UNKNOWN, None, None)
    red = min(same_row, key=lambda item: item.center[0])
    route = ROUTE_2 if red.center[0] < green.center[0] else ROUTE_1
    return EndpointDecision(route, green, red)


class EndpointRouteLatch:
    """Default to F1 and permanently latch F2 after one valid F2 frame."""

    def __init__(self) -> None:
        self.selected_final = 1
        self.locked = False
        self.last_decision = UNKNOWN
        self.confidence = 0.0

    def reset(self) -> None:
        self.selected_final = 1
        self.locked = False
        self.last_decision = UNKNOWN
        self.confidence = 0.0

    def update(self, decision: EndpointDecision) -> bool:
        self.last_decision = decision.route
        if decision.green is not None and decision.red is not None:
            self.confidence = min(decision.green.confidence, decision.red.confidence)
        else:
            self.confidence = 0.0
        changed = False
        if not self.locked and decision.route == ROUTE_2:
            self.selected_final = 2
            self.locked = True
            changed = True
        return changed


def event_s(route: Route, event_id: str) -> float:
    for point in route.waypoints:
        if event_id in point.fsm_event_ids.split("|"):
            return point.s_m
    raise ValueError(f"route has no event {event_id}")


def smoothstep(value: float) -> float:
    value = max(0.0, min(1.0, float(value)))
    return value * value * (3.0 - 2.0 * value)


def blended_final_route(
    reference_f1: Route,
    target_f2: Route,
    transition_length_m: float = 8.0,
    reference_transition_s_m: float | None = None,
) -> Route:
    """Keep the current F1 pose and merge continuously into F2.

    The two authored variants can have slightly different accumulated s values
    because their parking branches differ.  END_LANE_DECISION_START is used as
    the common origin, then the reference F1 position is sampled with the same
    distance offset as each F2 waypoint.  A late camera decision starts the
    blend at the vehicle's current F1 progress instead of jumping to a blend
    that began earlier in the section.
    """
    if not math.isfinite(transition_length_m) or transition_length_m <= 0.0:
        raise ValueError("endpoint transition length must be positive")
    reference_start = event_s(reference_f1, "END_LANE_DECISION_START")
    target_start = event_s(target_f2, "END_LANE_DECISION_START")
    if reference_transition_s_m is None:
        reference_transition_s_m = reference_start
    if not math.isfinite(reference_transition_s_m):
        raise ValueError("endpoint transition start must be finite")
    transition_offset = max(0.0, reference_transition_s_m - reference_start)
    blended: list[Waypoint] = []
    for index, target in enumerate(target_f2.waypoints):
        offset = target.s_m - target_start
        reference_s = reference_start + offset
        reference_x, reference_y = reference_f1.position_at_s(reference_s)
        weight = smoothstep((offset - transition_offset) / transition_length_m)
        x_m = reference_x + weight * (target.x_m - reference_x)
        y_m = reference_y + weight * (target.y_m - reference_y)
        blended.append(
            Waypoint(
                index=index,
                s_m=target.s_m,
                x_m=x_m,
                y_m=y_m,
                target_speed_mps=target.target_speed_mps,
                mission=target.mission,
                direction=target.direction,
                fsm_zone=target.fsm_zone,
                fsm_event_ids=target.fsm_event_ids,
                fsm_actions=target.fsm_actions,
                fsm_conditions=target.fsm_conditions,
                fsm_hold_sec=target.fsm_hold_sec,
                direction_source=target.direction_source,
            )
        )
    return Route(blended)
