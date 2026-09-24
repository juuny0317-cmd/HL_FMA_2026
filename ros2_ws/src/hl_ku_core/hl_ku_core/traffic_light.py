"""Pure traffic-light class mapping and target-selection helpers."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
from typing import Mapping, Sequence

from .mission import (
    LIGHT_GREEN,
    LIGHT_LEFT,
    LIGHT_RED,
    LIGHT_UNKNOWN,
    LIGHT_YELLOW,
)


TRAFFIC_YOLO_NAMES = ("green", "left", "red", "yellow")
INTEGRATED_YOLO_NAMES = (
    "green",
    "left",
    "red",
    "yellow",
    "s_obstacle",
    "pedestrian",
    "end_point",
)
# Kept for callers that imported the old public constant. It now describes the
# competition model, while TRAFFIC_YOLO_NAMES describes the traffic subset.
EXPECTED_YOLO_NAMES = INTEGRATED_YOLO_NAMES
FORBIDDEN_YOLO_ALIASES = frozenset(("green_light", "s-oblstacle", "first"))
YOLO_TO_ROS = {
    "green": LIGHT_GREEN,
    "left": LIGHT_LEFT,
    "red": LIGHT_RED,
    "yellow": LIGHT_YELLOW,
}

SIGNAL_RED_BIT = 1 << 0
SIGNAL_YELLOW_BIT = 1 << 1
SIGNAL_GREEN_BIT = 1 << 2
SIGNAL_LEFT_BIT = 1 << 3
TRAFFIC_SIGNAL_BITS = {
    "red": SIGNAL_RED_BIT,
    "yellow": SIGNAL_YELLOW_BIT,
    "green": SIGNAL_GREEN_BIT,
    "left": SIGNAL_LEFT_BIT,
}
ALL_TRAFFIC_SIGNAL_BITS = sum(TRAFFIC_SIGNAL_BITS.values())


@dataclass(frozen=True)
class Detection:
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
    def area(self) -> float:
        return max(0.0, self.x2 - self.x1) * max(0.0, self.y2 - self.y1)


@dataclass(frozen=True)
class TrafficVote:
    frame_count: int
    red_votes: int
    yellow_votes: int
    green_votes: int
    left_votes: int
    confirmed_mask: int

    def confirmed(self, signal_bit: int) -> bool:
        return bool(self.confirmed_mask & signal_bit)


class TrafficSignalVoter:
    """Class-specific N-of-M voting over new YOLO inference frames."""

    def __init__(
        self,
        window_frames: int = 1,
        *,
        red_required_votes: int = 1,
        yellow_required_votes: int = 1,
        green_required_votes: int = 1,
        left_required_votes: int = 1,
    ) -> None:
        self.window_frames = int(window_frames)
        self.required_votes = {
            SIGNAL_RED_BIT: int(red_required_votes),
            SIGNAL_YELLOW_BIT: int(yellow_required_votes),
            SIGNAL_GREEN_BIT: int(green_required_votes),
            SIGNAL_LEFT_BIT: int(left_required_votes),
        }
        if self.window_frames <= 0:
            raise ValueError("traffic vote window must be positive")
        if any(
            required <= 0 or required > self.window_frames
            for required in self.required_votes.values()
        ):
            raise ValueError("traffic required votes must be within the window")
        self._history: deque[int] = deque(maxlen=self.window_frames)

    def reset(self) -> None:
        self._history.clear()

    def update(self, signal_mask: int) -> TrafficVote:
        self._history.append(int(signal_mask) & ALL_TRAFFIC_SIGNAL_BITS)
        return self.result

    @property
    def result(self) -> TrafficVote:
        counts = {
            bit: sum(bool(mask & bit) for mask in self._history)
            for bit in self.required_votes
        }
        confirmed_mask = 0
        # A short partial window must never be mistaken for five camera frames.
        if len(self._history) == self.window_frames:
            for bit, required in self.required_votes.items():
                if counts[bit] >= required:
                    confirmed_mask |= bit
        return TrafficVote(
            frame_count=len(self._history),
            red_votes=counts[SIGNAL_RED_BIT],
            yellow_votes=counts[SIGNAL_YELLOW_BIT],
            green_votes=counts[SIGNAL_GREEN_BIT],
            left_votes=counts[SIGNAL_LEFT_BIT],
            confirmed_mask=confirmed_mask,
        )


class TrafficSignalVoteSession:
    """Reset frame votes across course contexts and inference stalls."""

    def __init__(self, voter: TrafficSignalVoter, stale_sec: float = 0.5) -> None:
        self.voter = voter
        self.stale_sec = float(stale_sec)
        if not math.isfinite(self.stale_sec) or self.stale_sec <= 0.0:
            raise ValueError("traffic vote stale timeout must be positive")
        self._context: tuple[str, str] | None = None
        self._last_frame_sec: float | None = None

    def reset(self) -> None:
        self.voter.reset()
        self._last_frame_sec = None

    def set_context(self, section_name: str, yolo_mode: str) -> None:
        context = (str(section_name).strip(), str(yolo_mode).strip().upper())
        if context != self._context:
            self._context = context
            self.reset()

    def update(self, signal_mask: int, now_sec: float) -> TrafficVote:
        if (
            not math.isfinite(now_sec)
            or (
                self._last_frame_sec is not None
                and (
                    now_sec - self._last_frame_sec > self.stale_sec
                    or now_sec < self._last_frame_sec
                )
            )
        ):
            self.reset()
        self._last_frame_sec = now_sec
        return self.voter.update(signal_mask)

    def result(self, now_sec: float) -> TrafficVote:
        if (
            self._last_frame_sec is not None
            and (
                not math.isfinite(now_sec)
                or now_sec - self._last_frame_sec > self.stale_sec
                or now_sec < self._last_frame_sec
            )
        ):
            self.reset()
        return self.voter.result


def normalize_model_names(
    names: Mapping[int, str] | Sequence[str],
    required_names: Sequence[str] = INTEGRATED_YOLO_NAMES,
) -> tuple[str, ...]:
    """Validate canonical names without assuming a model class-ID order."""
    if isinstance(names, Mapping):
        try:
            ordered = tuple(str(names[index]).strip().lower() for index in range(len(names)))
        except (KeyError, TypeError) as error:
            raise ValueError("model.names must use contiguous integer class IDs") from error
    else:
        ordered = tuple(str(name).strip().lower() for name in names)
    if len(ordered) != len(set(ordered)):
        raise ValueError(f"duplicate YOLO class names are unsafe: {ordered}")
    aliases = FORBIDDEN_YOLO_ALIASES.intersection(ordered)
    if aliases:
        raise ValueError(f"non-canonical YOLO aliases are not allowed: {sorted(aliases)}")
    required = tuple(str(name).strip().lower() for name in required_names)
    missing = sorted(set(required) - set(ordered))
    unexpected = sorted(set(ordered) - set(required))
    if missing or unexpected:
        raise ValueError(
            "unsafe YOLO classes: expected canonical set "
            f"{required}, received {ordered}; missing={missing}, unexpected={unexpected}"
        )
    return ordered


def ros_light_for_class_name(
    class_name: str | None,
    confidence: float,
    confidence_threshold: float,
) -> int:
    """Map a YOLO class name to ROS; numeric class IDs are never accepted here."""
    if class_name is None or not math.isfinite(confidence):
        return LIGHT_UNKNOWN
    if confidence < confidence_threshold:
        return LIGHT_UNKNOWN
    return YOLO_TO_ROS.get(str(class_name).strip().lower(), LIGHT_UNKNOWN)


def intersection_over_union(first: Detection, second: Detection) -> float:
    x1 = max(first.x1, second.x1)
    y1 = max(first.y1, second.y1)
    x2 = min(first.x2, second.x2)
    y2 = min(first.y2, second.y2)
    intersection = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    union = first.area + second.area - intersection
    return intersection / union if union > 0.0 else 0.0


def valid_traffic_detections(
    detections: Sequence[Detection],
    image_width: int,
    image_height: int,
    traffic_roi: Sequence[float],
    minimum_box_area_px2: float,
    edge_margin_ratio: float,
    preferred_point: tuple[float, float],
    maximum_target_distance_ratio: float,
) -> tuple[Detection, ...]:
    """Keep every lane-relevant traffic signal so simultaneous states survive."""
    if image_width <= 0 or image_height <= 0 or len(traffic_roi) != 4:
        return ()
    rx1, ry1, rx2, ry2 = (float(value) for value in traffic_roi)
    margin_x = image_width * max(0.0, edge_margin_ratio)
    margin_y = image_height * max(0.0, edge_margin_ratio)
    preferred_x = preferred_point[0] * image_width
    preferred_y = preferred_point[1] * image_height
    diagonal = math.hypot(image_width, image_height)
    valid = []
    for detection in detections:
        if detection.class_name not in TRAFFIC_YOLO_NAMES:
            continue
        center_x, center_y = detection.center
        if not (
            rx1 * image_width <= center_x <= rx2 * image_width
            and ry1 * image_height <= center_y <= ry2 * image_height
        ):
            continue
        if detection.area < minimum_box_area_px2:
            continue
        if (
            detection.x1 < margin_x
            or detection.y1 < margin_y
            or detection.x2 > image_width - margin_x
            or detection.y2 > image_height - margin_y
        ):
            continue
        distance = math.hypot(center_x - preferred_x, center_y - preferred_y) / max(
            diagonal, 1.0
        )
        if distance <= maximum_target_distance_ratio:
            valid.append(detection)
    return tuple(valid)


def traffic_signal_mask(
    detections: Sequence[Detection], confidence_threshold: float
) -> int:
    """Encode every confident traffic class; red and left may coexist."""
    mask = 0
    for detection in detections:
        if detection.confidence >= confidence_threshold:
            mask |= TRAFFIC_SIGNAL_BITS.get(detection.class_name, 0)
    return mask


def select_target_detection(
    detections: Sequence[Detection],
    image_width: int,
    image_height: int,
    traffic_roi: Sequence[float],
    minimum_box_area_px2: float,
    edge_margin_ratio: float,
    preferred_point: tuple[float, float],
    maximum_target_distance_ratio: float,
) -> Detection | None:
    """Choose one lane-relevant, trackable target instead of merging all boxes."""
    valid = valid_traffic_detections(
        detections,
        image_width,
        image_height,
        traffic_roi,
        minimum_box_area_px2,
        edge_margin_ratio,
        preferred_point,
        maximum_target_distance_ratio,
    )
    if not valid:
        return None
    preferred_x = preferred_point[0] * image_width
    preferred_y = preferred_point[1] * image_height
    diagonal = math.hypot(image_width, image_height)
    candidates: list[tuple[float, Detection]] = []
    for detection in valid:
        center_x, center_y = detection.center
        distance = math.hypot(center_x - preferred_x, center_y - preferred_y) / max(
            diagonal, 1.0
        )
        # Position consistency dominates a small confidence difference. Area is
        # only a weak tie-breaker so a large off-lane signal does not take over.
        area_ratio = detection.area / max(float(image_width * image_height), 1.0)
        score = detection.confidence - 1.25 * distance + min(area_ratio, 0.05)
        candidates.append((score, detection))
    if not candidates:
        return None
    selected = max(candidates, key=lambda item: item[0])[1]

    # Red and a lit left arrow can be co-located. Only an overlapping LEFT box
    # may override the selected state; detections on other signal heads cannot.
    overlapping_left = [
        detection
        for _, detection in candidates
        if detection.class_name == "left"
        and intersection_over_union(selected, detection) >= 0.50
    ]
    if overlapping_left:
        return max(overlapping_left, key=lambda detection: detection.confidence)
    return selected


def select_semantic_detection(
    detections: Sequence[Detection],
    class_name: str,
    image_width: int,
    image_height: int,
    roi: Sequence[float],
    confidence_threshold: float,
    minimum_box_area_px2: float,
) -> Detection | None:
    """Return the strongest in-ROI detection for one non-traffic class."""
    if image_width <= 0 or image_height <= 0 or len(roi) != 4:
        return None
    rx1, ry1, rx2, ry2 = (float(value) for value in roi)
    if not (0.0 <= rx1 < rx2 <= 1.0 and 0.0 <= ry1 < ry2 <= 1.0):
        return None
    candidates = []
    for detection in detections:
        if detection.class_name != class_name:
            continue
        if detection.confidence < confidence_threshold:
            continue
        if detection.area < minimum_box_area_px2:
            continue
        center_x, center_y = detection.center
        if not (
            rx1 * image_width <= center_x <= rx2 * image_width
            and ry1 * image_height <= center_y <= ry2 * image_height
        ):
            continue
        candidates.append(detection)
    return max(candidates, key=lambda detection: detection.confidence, default=None)


@dataclass
class DetectionStabilizer:
    """Require consecutive agreement, emitting UNKNOWN during transitions."""

    required_confirmations: int = 2
    _candidate: int = LIGHT_UNKNOWN
    _count: int = 0

    def update(self, value: int) -> int:
        confirmations = max(1, int(self.required_confirmations))
        if value == LIGHT_UNKNOWN:
            self.reset()
            return LIGHT_UNKNOWN
        if value != self._candidate:
            self._candidate = value
            self._count = 1
        else:
            self._count += 1
        return value if self._count >= confirmations else LIGHT_UNKNOWN

    def reset(self) -> None:
        self._candidate = LIGHT_UNKNOWN
        self._count = 0
