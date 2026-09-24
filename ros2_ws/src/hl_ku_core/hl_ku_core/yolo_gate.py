"""Select only the competition classes needed in the active route zone."""

from __future__ import annotations


MODE_CLASSES = {
    "TRAFFIC": frozenset({"green", "left", "red", "yellow"}),
    "PEDESTRIAN": frozenset({"pedestrian"}),
    "OFF": frozenset(),
}


def mode_for_route_zone(zone: str) -> str:
    zone = str(zone).strip().upper()
    if zone == "TRAFFIC":
        return "TRAFFIC"
    if zone == "DUMMY":
        return "PEDESTRIAN"
    return "OFF"


def allowed_classes(mode: str) -> frozenset[str]:
    return MODE_CLASSES.get(str(mode).strip().upper(), MODE_CLASSES["OFF"])


def class_ids_for_mode(names: tuple[str, ...], mode: str) -> list[int]:
    allowed = allowed_classes(mode)
    return [index for index, name in enumerate(names) if name in allowed]
