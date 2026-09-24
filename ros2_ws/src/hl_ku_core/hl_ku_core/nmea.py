"""NMEA parsing needed by the UM982 USB serial node."""

from __future__ import annotations

import datetime as dt
import math
from dataclasses import dataclass
from typing import Optional, Union


KNOT_TO_MPS = 0.5144444444444445


@dataclass(frozen=True)
class Gga:
    utc: str
    latitude_deg: float
    longitude_deg: float
    fix_quality: int
    satellites: int
    hdop: float
    altitude_m: float
    correction_age_sec: float


@dataclass(frozen=True)
class Rmc:
    utc: str
    valid: bool
    latitude_deg: float
    longitude_deg: float
    ground_speed_mps: float
    track_true_deg: float


@dataclass(frozen=True)
class Hdt:
    heading_true_deg: float
    valid: bool = True


NmeaMessage = Union[Gga, Rmc, Hdt]


def nmea_checksum(payload: str) -> int:
    value = 0
    for character in payload:
        value ^= ord(character)
    return value


def validate_sentence(sentence: str) -> bool:
    sentence = sentence.strip()
    if not sentence.startswith("$") or "*" not in sentence:
        return False
    payload, checksum_text = sentence[1:].rsplit("*", 1)
    try:
        expected = int(checksum_text[:2], 16)
    except ValueError:
        return False
    return nmea_checksum(payload) == expected


def _safe_float(value: str, default: float = math.nan) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _safe_int(value: str, default: int = 0) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def _coordinate(value: str, hemisphere: str, degree_digits: int) -> float:
    if not value or len(value) <= degree_digits:
        return math.nan
    valid_hemispheres = ("N", "S") if degree_digits == 2 else ("E", "W")
    if hemisphere not in valid_hemispheres:
        return math.nan
    degrees = float(value[:degree_digits])
    minutes = float(value[degree_digits:])
    maximum_degrees = 90.0 if degree_digits == 2 else 180.0
    if not (0.0 <= degrees <= maximum_degrees and 0.0 <= minutes < 60.0):
        return math.nan
    if degrees == maximum_degrees and minutes > 0.0:
        return math.nan
    result = degrees + minutes / 60.0
    if hemisphere in ("S", "W"):
        result = -result
    return result


def parse_sentence(sentence: str, require_checksum: bool = True) -> Optional[NmeaMessage]:
    sentence = sentence.strip()
    checksum_valid = validate_sentence(sentence)
    if require_checksum and not checksum_valid:
        return None
    if not sentence.startswith("$"):
        return None
    payload = sentence[1:].split("*", 1)[0]
    fields = payload.split(",")
    kind = fields[0][-3:] if fields else ""
    try:
        if kind == "GGA" and len(fields) >= 14:
            return Gga(
                utc=fields[1],
                latitude_deg=_coordinate(fields[2], fields[3], 2),
                longitude_deg=_coordinate(fields[4], fields[5], 3),
                fix_quality=_safe_int(fields[6]),
                satellites=_safe_int(fields[7]),
                hdop=_safe_float(fields[8]),
                altitude_m=_safe_float(fields[9]),
                correction_age_sec=_safe_float(fields[13]),
            )
        if kind == "RMC" and len(fields) >= 9:
            return Rmc(
                utc=fields[1],
                valid=fields[2] == "A",
                latitude_deg=_coordinate(fields[3], fields[4], 2),
                longitude_deg=_coordinate(fields[5], fields[6], 3),
                ground_speed_mps=_safe_float(fields[7], 0.0) * KNOT_TO_MPS,
                track_true_deg=_safe_float(fields[8], 0.0),
            )
        if kind == "HDT" and len(fields) >= 3:
            heading = _safe_float(fields[1])
            if math.isfinite(heading):
                return Hdt(heading_true_deg=heading % 360.0)
        # UM982's documented dual-antenna true-heading sentence is GNTHS.
        if kind == "THS" and len(fields) >= 3:
            heading = _safe_float(fields[1])
            valid = fields[2] not in ("", "V") and math.isfinite(heading)
            return Hdt(
                heading_true_deg=(heading % 360.0) if valid else math.nan,
                valid=valid,
            )
    except (ValueError, IndexError):
        return None
    return None


def _nmea_coordinate(value_deg: float, latitude: bool) -> tuple[str, str]:
    hemisphere = ("N" if value_deg >= 0.0 else "S") if latitude else (
        "E" if value_deg >= 0.0 else "W"
    )
    absolute = abs(value_deg)
    degrees = int(absolute)
    minutes = (absolute - degrees) * 60.0
    if latitude:
        return f"{degrees:02d}{minutes:08.5f}", hemisphere
    return f"{degrees:03d}{minutes:08.5f}", hemisphere


def make_gga(latitude_deg: float, longitude_deg: float, altitude_m: float = 0.0) -> str:
    """Build the rover-position GGA sentence sent to an NTRIP caster."""
    now = dt.datetime.now(dt.timezone.utc)
    utc = now.strftime("%H%M%S") + f".{now.microsecond // 10000:02d}"
    lat, ns = _nmea_coordinate(latitude_deg, True)
    lon, ew = _nmea_coordinate(longitude_deg, False)
    payload = (
        f"GPGGA,{utc},{lat},{ns},{lon},{ew},1,12,1.0,{altitude_m:.3f},M,0.0,M,,"
    )
    return f"${payload}*{nmea_checksum(payload):02X}\r\n"
