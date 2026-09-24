#!/usr/bin/env python3
"""Build the single FSM-tagged route bundle used by the Mando TUI.

The source parallel-CSV files are geometry previews with zero target speeds. This
bundle preserves their geometry and authored FSM event metadata; the TUI
materializes a selected variant at the current pose and applies its chosen
low-speed test value before launch.
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
import sys
from typing import Sequence


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws/src/hl_ku_core"))
from hl_ku_core.route import FSM_ZONE_TO_MISSION

SOURCE_DIR = ROOT / "ros2_ws/src/hl_ku_core/routes/course_07_parking_overlay"
OUTPUT = ROOT / "ros2_ws/src/hl_ku_core/routes/course_07_mission.csv"
VARIANTS = (
    "p1_t1_f1",
    "p1_t1_f2",
    "p1_t2_f1",
    "p1_t2_f2",
    "p2_t1_f1",
    "p2_t1_f2",
    "p2_t2_f1",
    "p2_t2_f2",
)
FIELDS = (
    "variant_id",
    "s_m",
    "x_m",
    "y_m",
    "target_speed_mps",
    "mission",
    "direction",
    "fsm_zone",
    "fsm_event_ids",
    "fsm_actions",
    "fsm_conditions",
    "fsm_hold_sec",
    "direction_source",
)


def build_rows() -> list[dict[str, str]]:
    output: list[dict[str, str]] = []
    for variant_id in VARIANTS:
        source = SOURCE_DIR / f"course_07_{variant_id}_parallel_csv.csv"
        with source.open(newline="", encoding="utf-8") as stream:
            reader = csv.DictReader(stream)
            required = set(FIELDS[1:]) - {"mission"}
            missing = required.difference(reader.fieldnames or ())
            if missing:
                raise ValueError(f"{source} missing columns: {', '.join(sorted(missing))}")
            rows = list(reader)
        if len(rows) < 2:
            raise ValueError(f"{source} has fewer than two waypoints")
        previous_s = float("-inf")
        for index, row in enumerate(rows, start=2):
            zone = (row.get("fsm_zone") or "").strip().upper()
            if zone not in FSM_ZONE_TO_MISSION:
                raise ValueError(f"{source}:{index} unknown fsm_zone {zone!r}")
            s_m = float(row["s_m"])
            if s_m <= previous_s:
                raise ValueError(f"{source}:{index} s_m is not strictly increasing")
            previous_s = s_m
            normalized = {key: (row.get(key) or "").strip() for key in FIELDS[1:]}
            normalized["mission"] = FSM_ZONE_TO_MISSION[zone]
            normalized["fsm_zone"] = zone
            normalized["variant_id"] = variant_id
            output.append(normalized)
    return output


def render() -> str:
    from io import StringIO

    stream = StringIO(newline="")
    writer = csv.DictWriter(stream, fieldnames=FIELDS, lineterminator="\n")
    writer.writeheader()
    writer.writerows(build_rows())
    return stream.getvalue()


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="fail if output is stale")
    parser.add_argument("--output", type=Path, default=OUTPUT)
    options = parser.parse_args(argv)
    output = options.output.expanduser()
    expected = render()
    if options.check:
        if not output.is_file() or output.read_text(encoding="utf-8") != expected:
            print(f"stale or missing: {output}")
            return 1
        print(f"current: {output}")
        return 0
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_suffix(output.suffix + ".tmp")
    temporary.write_text(expected, encoding="utf-8")
    temporary.replace(output)
    print(f"wrote {len(build_rows())} rows across {len(VARIANTS)} variants: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
