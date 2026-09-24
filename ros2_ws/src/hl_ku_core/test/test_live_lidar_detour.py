"""The S-route detour needs a real scan cluster; the preview obstacle is separate."""

import math
from pathlib import Path

import yaml

from hl_ku_core.detour_following import DetourFollowResult, follow_detour
from hl_ku_core.detour_gate import select_detour_command
from hl_ku_core.local_detour import CircleObstacle, plan_local_detour
from hl_ku_core.route import Route
from hl_ku_core.scan_obstacle_mapping import (
    MapperSettings,
    VehiclePose,
    clustered_obstacle_discs_in_map,
)


PACKAGE = Path(__file__).resolve().parents[1]


def test_s_route_only_detours_for_an_obstacle_cluster_in_the_live_scan():
    config = yaml.safe_load((PACKAGE / "config" / "system.yaml").read_text())
    mount = config["scan_obstacle_mapper"]["ros__parameters"]
    assert mount["lidar_x_m"] == 1.25
    assert mount["lidar_y_m"] == 0.0
    settings = MapperSettings(
        lidar_x_m=mount["lidar_x_m"],
        lidar_y_m=mount["lidar_y_m"],
        lidar_yaw_rad=math.radians(mount["lidar_yaw_deg"]),
    )
    route = Route.load_csv(PACKAGE / "routes" / "mando" / "seg_s_full.csv")
    progress_s = 2.0
    vehicle_x, vehicle_y = route.position_at_s(progress_s)
    vehicle_yaw = route.heading_at_s(progress_s)
    pose = VehiclePose(vehicle_x, vehicle_y, vehicle_yaw)
    obstacle_x, obstacle_y = route.position_at_s(10.0)
    lidar_x = vehicle_x + 1.25 * math.cos(vehicle_yaw)
    lidar_y = vehicle_y + 1.25 * math.sin(vehicle_yaw)
    distance = math.hypot(obstacle_x - lidar_x, obstacle_y - lidar_y)
    sensor_angle = (
        math.atan2(obstacle_y - lidar_y, obstacle_x - lidar_x)
        - vehicle_yaw - settings.lidar_yaw_rad + math.pi
    ) % (2.0 * math.pi) - math.pi
    angle_increment = 2.0 * math.pi / 1800
    center_index = round((sensor_angle + math.pi) / angle_increment)
    ranges = [math.inf] * 1800

    def map_scan():
        return clustered_obstacle_discs_in_map(
            ranges, -math.pi, angle_increment, 0.15, 25.0, pose, settings
        )

    assert map_scan() == ()
    assert plan_local_detour(route, progress_s, ()).status == "NO_OBSTACLE"
    no_obstacle_command = select_detour_command(
        enabled=True, in_s_obstacle=True, engaged=False,
        speed_mps=0.25, route_steering_rad=0.05,
        legacy_steering_offset_rad=0.0, lidar_fresh=True,
        obstacle_in_path=False, obstacle_distance_m=math.inf,
        follow=DetourFollowResult("UNAVAILABLE"),
        approach_speed_limit_mps=0.30,
    )
    assert not no_obstacle_command.engaged
    assert not no_obstacle_command.brake
    assert no_obstacle_command.steering_rad == 0.05

    # A vehicle-sized object appears in the actual LaserScan input.
    for index in range(center_index - 5, center_index + 6):
        ranges[index] = distance
    centers = map_scan()
    assert len(centers) == 1
    assert math.dist(centers[0][:2], (obstacle_x, obstacle_y)) < 0.05
    assert centers[0][2] >= 0.40
    detour = plan_local_detour(
        route, progress_s, (CircleObstacle(*centers[0]),)
    )
    assert detour.status == "CANDIDATE"
    assert follow_detour(
        detour.points, vehicle_x, vehicle_y, vehicle_yaw
    ).state == "APPROACH"
    first, second = detour.points[:2]
    entry_yaw = math.atan2(second[1] - first[1], second[0] - first[0])
    follow = follow_detour(detour.points, *first, entry_yaw, speed_mps=0.25)
    assert follow.state == "TRACK"
    obstacle_command = select_detour_command(
        enabled=True, in_s_obstacle=True, engaged=False,
        speed_mps=0.25, route_steering_rad=0.05,
        legacy_steering_offset_rad=0.0, lidar_fresh=True,
        obstacle_in_path=True, obstacle_distance_m=distance,
        follow=follow,
        approach_speed_limit_mps=0.30,
    )
    assert obstacle_command.engaged
    assert not obstacle_command.brake
    assert obstacle_command.steering_rad == follow.steering_rad
