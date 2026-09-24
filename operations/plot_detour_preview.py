#!/usr/bin/env python3
"""Render the exact preview planner result without ROS or a connected LiDAR."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys

import matplotlib.pyplot as plt
from matplotlib.patches import Circle


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "ros2_ws" / "src" / "hl_ku_core"))

from hl_ku_core.local_detour import CircleObstacle, DetourSettings, plan_local_detour  # noqa: E402
from hl_ku_core.route import Route  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--route",
        type=Path,
        default=REPO_ROOT
        / "ros2_ws/src/hl_ku_core/routes/course_07_vehicle.csv",
    )
    parser.add_argument("--progress-s", type=float, default=200.0)
    parser.add_argument("--obstacle-s", type=float, default=210.0)
    parser.add_argument("--obstacle-lateral", type=float, default=0.0)
    parser.add_argument("--obstacle-radius", type=float, default=0.25)
    parser.add_argument(
        "--output",
        type=Path,
        default=REPO_ROOT / "docs/images/local_detour_preview.png",
    )
    args = parser.parse_args()
    route = Route.load_csv(args.route)
    settings = DetourSettings()
    ox, oy = route.position_at_s(args.obstacle_s)
    heading = route.heading_at_s(args.obstacle_s)
    obstacle = CircleObstacle(
        ox - math.sin(heading) * args.obstacle_lateral,
        oy + math.cos(heading) * args.obstacle_lateral,
        args.obstacle_radius,
    )
    result = plan_local_detour(route, args.progress_s, (obstacle,), settings)
    end_s = min(route.waypoints[-1].s_m, args.obstacle_s + 9.0)
    start_s = max(route.waypoints[0].s_m, args.obstacle_s - 9.0)
    count = max(2, math.ceil((end_s - start_s) / 0.1))
    path_points = [
        route.position_at_s(start_s + (end_s - start_s) * index / count)
        for index in range(count + 1)
    ]
    fig, ax = plt.subplots(figsize=(8.8, 6.0), dpi=160)
    ax.plot(
        [point[0] for point in path_points],
        [point[1] for point in path_points],
        color="#d39700",
        linewidth=2.4,
        label="Global route",
    )
    if result.points:
        ax.plot(
            [point[0] for point in result.points],
            [point[1] for point in result.points],
            color="#008fc7",
            linewidth=3.0,
            label="Local detour candidate",
        )
        first, last = result.points[0], result.points[-1]
        ax.scatter(*first, color="#009445", s=95, zorder=5, label="Leave route")
        ax.scatter(*last, color="#2866d9", s=95, zorder=5, label="Rejoin route")
    ax.add_patch(
        Circle(
            (obstacle.x_m, obstacle.y_m),
            obstacle.radius_m + settings.vehicle_half_width_m + settings.safety_margin_m,
            facecolor="#f06464",
            edgecolor="none",
            alpha=0.17,
            label="Obstacle + vehicle clearance",
        )
    )
    ax.add_patch(
        Circle(
            (obstacle.x_m, obstacle.y_m),
            obstacle.radius_m,
            facecolor="#e84444",
            edgecolor="#b31c1c",
            linewidth=1,
            zorder=6,
            label="Mock obstacle",
        )
    )
    ax.set_title(f"HL_KU detour preview  |  {result.status}  |  {result.side}")
    ax.set_xlabel("Map X (m)")
    ax.set_ylabel("Map Y (m)")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(alpha=0.25)
    ax.legend(loc="best", fontsize=8)
    fig.tight_layout()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output)
    plt.close(fig)
    print(f"{result.status} {result.side}: {args.output}")
    return 0 if result.status == "CANDIDATE" else 1


if __name__ == "__main__":
    raise SystemExit(main())
