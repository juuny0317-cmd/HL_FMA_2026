"""Per-inference pedestrian evidence and independent stop/clear voting."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
from typing import Sequence

from .traffic_light import Detection


STOP_PHASES = frozenset(("START", "INSIDE", "END"))
PEDESTRIAN_PHASES = frozenset(("PRE_ENABLE", *STOP_PHASES, "AFTER_END"))


def pedestrian_stop_candidates(
    detections: Sequence[Detection],
    image_width: int,
    image_height: int,
    roi: Sequence[float],
    minimum_box_height_ratio: float,
) -> tuple[Detection, ...]:
    """Keep pedestrians in the configured image region once they are close enough."""
    if image_width <= 0 or image_height <= 0:
        raise ValueError("image dimensions must be positive")
    if len(roi) != 4:
        raise ValueError("pedestrian ROI must contain four normalized values")
    if (
        not math.isfinite(minimum_box_height_ratio)
        or not 0.0 <= minimum_box_height_ratio <= 1.0
    ):
        raise ValueError("minimum pedestrian box height ratio must be in [0, 1]")
    x0, y0, x1, y1 = (float(value) for value in roi)
    if not all(math.isfinite(value) for value in (x0, y0, x1, y1)):
        raise ValueError("pedestrian ROI values must be finite")
    if not (0.0 <= x0 <= x1 <= 1.0 and 0.0 <= y0 <= y1 <= 1.0):
        raise ValueError("pedestrian ROI must be ordered within [0, 1]")

    minimum_height = minimum_box_height_ratio * image_height
    return tuple(
        detection
        for detection in detections
        if detection.class_name == "pedestrian"
        and x0 * image_width <= detection.center[0] <= x1 * image_width
        and y0 * image_height <= detection.center[1] <= y1 * image_height
        and max(0.0, detection.y2 - detection.y1) >= minimum_height
    )


def pedestrian_evidence(
    detections: Sequence[Detection], confidence_threshold: float
) -> tuple[bool, float]:
    """Use only class and confidence; do not infer whether a person is in-road."""
    if not math.isfinite(confidence_threshold) or not 0.0 <= confidence_threshold <= 1.0:
        raise ValueError("pedestrian confidence threshold must be in [0, 1]")
    confidences = [
        detection.confidence
        for detection in detections
        if detection.class_name == "pedestrian"
        and math.isfinite(detection.confidence)
    ]
    maximum = max(confidences, default=0.0)
    return bool(confidences) and maximum >= confidence_threshold, maximum


@dataclass(frozen=True)
class PedestrianVote:
    history: tuple[bool, ...]
    stop_votes: int
    stop_confirmed: bool
    clear_count: int
    clear_confirmed: bool
    stale: bool


class PedestrianVoteSession:
    """One update per completed inference; timer reads never add evidence."""

    def __init__(
        self,
        window_frames: int = 5,
        required_votes: int = 3,
        clear_consecutive_frames: int = 5,
        stale_sec: float = 0.5,
    ) -> None:
        self.window_frames = int(window_frames)
        self.required_votes = int(required_votes)
        self.clear_consecutive_frames = int(clear_consecutive_frames)
        self.stale_sec = float(stale_sec)
        if self.window_frames <= 0 or not 0 < self.required_votes <= self.window_frames:
            raise ValueError("invalid pedestrian stop voting window")
        if self.clear_consecutive_frames <= 0:
            raise ValueError("pedestrian clear count must be positive")
        if not math.isfinite(self.stale_sec) or self.stale_sec <= 0.0:
            raise ValueError("pedestrian stale timeout must be positive")
        self._history: deque[bool] = deque(maxlen=self.window_frames)
        self._clear_count = 0
        self._clearing = False
        self._last_frame_sec: float | None = None
        self._context: tuple[str, str] | None = None
        self._phase = "OUTSIDE"

    def reset(self) -> None:
        self._history.clear()
        self._clear_count = 0
        self._clearing = False
        self._last_frame_sec = None

    def set_context(self, section_name: str, yolo_mode: str, phase: str) -> None:
        context = (str(section_name).strip(), str(yolo_mode).strip().upper())
        phase = str(phase).strip().upper()
        if context != self._context:
            self._context = context
            self.reset()
        elif self._phase not in STOP_PHASES and phase in STOP_PHASES:
            # PRE_ENABLE is diagnostic only; it cannot preload the stop vote.
            self._history.clear()
            self._last_frame_sec = None
        self._phase = phase

    def begin_clear(self) -> None:
        # No pre-stop inference may contribute to release.
        self._history.clear()
        self._clear_count = 0
        self._clearing = True
        self._last_frame_sec = None

    def _expire_if_stale(self, now_sec: float) -> bool:
        stale = self._last_frame_sec is None or (
            not math.isfinite(now_sec)
            or now_sec < self._last_frame_sec
            or now_sec - self._last_frame_sec > self.stale_sec
        )
        if stale:
            self._history.clear()
            self._clear_count = 0
            self._last_frame_sec = None
        return stale

    def update(self, detected: bool, now_sec: float) -> PedestrianVote:
        self._expire_if_stale(now_sec)
        if not math.isfinite(now_sec):
            return self.result(now_sec)
        self._last_frame_sec = now_sec
        if self._clearing:
            self._clear_count = (
                0 if detected else min(self.clear_consecutive_frames, self._clear_count + 1)
            )
        else:
            self._history.append(bool(detected))
        return self.result(now_sec)

    def result(self, now_sec: float) -> PedestrianVote:
        stale = self._expire_if_stale(now_sec)
        votes = sum(self._history)
        return PedestrianVote(
            history=tuple(self._history),
            stop_votes=votes,
            stop_confirmed=(
                not self._clearing
                and len(self._history) == self.window_frames
                and votes >= self.required_votes
            ),
            clear_count=self._clear_count,
            clear_confirmed=(
                self._clearing
                and self._clear_count >= self.clear_consecutive_frames
                and not stale
            ),
            stale=stale,
        )
