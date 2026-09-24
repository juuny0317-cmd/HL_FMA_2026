import math

import pytest
from builtin_interfaces.msg import Time
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker

from hl_ku_core.scan_obstacle_mapping import (
    MapperSettings,
    VehiclePose,
    clustered_obstacle_discs_in_map,
    clustered_obstacles_in_map,
)
from hl_ku_core.scan_obstacle_mapper_node import (
    _lidar_transform,
    _obstacle_markers,
    _scan_points_in_map,
)


def project(ranges, pose=VehiclePose(0.0, 0.0, 0.0), **settings):
    return clustered_obstacles_in_map(
        ranges,
        -0.2,
        0.1,
        0.10,
        25.0,
        pose,
        MapperSettings(minimum_cluster_points=3, **settings),
    )


def test_front_cluster_is_projected_into_map() -> None:
    obstacles = project([math.inf, 2.0, 2.0, 2.0, math.inf])
    assert len(obstacles) == 1
    assert obstacles[0][0] == pytest.approx(1.993, abs=0.01)
    assert obstacles[0][1] == pytest.approx(0.0, abs=0.01)


def test_rviz_transform_matches_mapper_extrinsics() -> None:
    transform = _lidar_transform(
        Time(sec=12),
        "map",
        "laser",
        VehiclePose(10.0, 20.0, math.pi / 2.0),
        MapperSettings(lidar_x_m=1.25, lidar_y_m=0.0, lidar_yaw_rad=-0.5),
    )
    assert transform.header.frame_id == "map"
    assert transform.child_frame_id == "laser"
    assert transform.transform.translation.x == pytest.approx(10.0)
    assert transform.transform.translation.y == pytest.approx(21.25)
    assert transform.transform.rotation.z == pytest.approx(
        math.sin((math.pi / 2.0 - 0.5) / 2.0)
    )


def test_rviz_obstacle_markers_show_cluster_radius_and_clear_old_markers() -> None:
    markers = _obstacle_markers(
        Time(sec=4), "map", ((1.0, 2.0, 0.4), (3.0, 4.0, 0.6))
    )
    assert markers.markers[0].action == Marker.DELETEALL
    assert len(markers.markers) == 3
    assert markers.markers[1].type == Marker.CYLINDER
    assert markers.markers[1].scale.x == pytest.approx(0.8)
    assert markers.markers[2].pose.position.y == pytest.approx(4.0)


def test_raw_scan_visualization_is_pretransformed_into_map() -> None:
    scan = LaserScan()
    scan.angle_min = 0.0
    scan.angle_increment = math.pi / 2.0
    scan.range_min = 0.1
    scan.range_max = 25.0
    scan.ranges = [2.0, 1.0, math.inf]
    points = _scan_points_in_map(
        scan,
        VehiclePose(10.0, 20.0, math.pi / 2.0),
        MapperSettings(lidar_x_m=1.0, lidar_y_m=0.0, lidar_yaw_rad=0.0),
    )
    assert len(points) == 2
    assert points[0] == pytest.approx((10.0, 23.0, 0.0))
    assert points[1] == pytest.approx((9.0, 21.0, 0.0))


def test_empty_valid_scan_produces_empty_obstacle_set() -> None:
    assert project([math.inf] * 5) == ()


def test_returns_below_half_meter_are_ignored() -> None:
    assert project([math.inf, 0.49, 0.49, 0.49, math.inf]) == ()
    obstacles = project([math.inf, 0.50, 0.50, 0.50, math.inf])
    assert len(obstacles) == 1


def test_visible_obstacle_width_increases_planner_radius() -> None:
    settings = MapperSettings(minimum_cluster_points=3)
    def discs(ranges, angle_min):
        return clustered_obstacle_discs_in_map(
            ranges, angle_min, 0.01, 0.1, 25.0,
            VehiclePose(0.0, 0.0, 0.0), settings,
        )
    narrow = discs([5.0] * 3, -0.01)
    wide = discs([5.0] * 21, -0.10)
    assert len(narrow) == len(wide) == 1
    assert narrow[0][2] == pytest.approx(0.40)
    assert wide[0][2] > 0.50


def test_short_cluster_and_rear_returns_are_rejected() -> None:
    assert project([2.0, 2.0, math.inf, 2.0, 2.0]) == ()
    rear = clustered_obstacles_in_map(
        [2.0, 2.0, 2.0],
        math.pi - 0.1,
        0.1,
        0.1,
        25.0,
        VehiclePose(0.0, 0.0, 0.0),
        MapperSettings(minimum_cluster_points=3),
    )
    assert rear == ()


def test_sensor_extrinsics_and_vehicle_pose_are_applied() -> None:
    obstacles = project(
        [2.0, 2.0, 2.0, math.inf, math.inf],
        pose=VehiclePose(10.0, 20.0, math.pi / 2.0),
        lidar_x_m=1.0,
        lidar_y_m=0.5,
        lidar_yaw_rad=0.0,
    )
    assert len(obstacles) == 1
    # Mean scan angle is -0.1 rad, then lidar offset, then vehicle +90 deg.
    expected_base_x = 1.0 + 2.0 * math.cos(0.1)
    expected_base_y = 0.5 - 2.0 * math.sin(0.1)
    assert obstacles[0][0] == pytest.approx(10.0 - expected_base_y, abs=0.01)
    assert obstacles[0][1] == pytest.approx(20.0 + expected_base_x, abs=0.01)


def test_lidar_yaw_is_applied_before_front_sector_filtering() -> None:
    obstacles = clustered_obstacles_in_map(
        [2.0, 2.0, 2.0],
        math.pi / 2.0 - 0.1,
        0.1,
        0.1,
        25.0,
        VehiclePose(0.0, 0.0, 0.0),
        MapperSettings(
            minimum_cluster_points=3,
            front_half_angle_rad=math.radians(20.0),
            lidar_yaw_rad=-math.pi / 2.0,
        ),
    )
    assert len(obstacles) == 1
    assert obstacles[0][0] == pytest.approx(1.993, abs=0.01)
    assert obstacles[0][1] == pytest.approx(0.0, abs=0.01)


def test_large_depth_jump_splits_clusters() -> None:
    obstacles = project(
        [2.0, 2.0, 5.0, 5.0, 5.0], maximum_cluster_gap_m=0.60
    )
    assert len(obstacles) == 1
    assert obstacles[0][0] == pytest.approx(4.98, abs=0.03)
    assert obstacles[0][1] == pytest.approx(0.50, abs=0.03)


@pytest.mark.parametrize(
    "ranges, increment",
    [([], 0.1), ([1.0], 0.0), ([1.0], math.inf)],
)
def test_invalid_scan_is_rejected(ranges, increment) -> None:
    with pytest.raises(ValueError):
        clustered_obstacles_in_map(
            ranges,
            0.0,
            increment,
            0.1,
            25.0,
            VehiclePose(0.0, 0.0, 0.0),
            MapperSettings(),
        )
