import math
from pathlib import Path

from hl_ku_core.detour_following import follow_detour
from hl_ku_core.local_detour import CircleObstacle, plan_local_detour
from hl_ku_core.route import Route


def straight_candidate():
    return tuple((10.0 + 0.2 * index, 0.0) for index in range(51))


def test_approach_track_and_rejoin_are_distinct():
    points = straight_candidate()
    assert follow_detour(points, 7.0, 0.0, 0.0).state == "APPROACH"
    tracked = follow_detour(points, 9.5, 0.0, 0.0)
    assert tracked.state == "TRACK"
    assert tracked.steering_rad == 0.0
    assert follow_detour(points, 20.1, 0.0, 0.0).state == "COMPLETE"


def test_bad_geometry_or_pose_cannot_command_detour_steering():
    points = straight_candidate()
    assert follow_detour(points, 12.0, 1.0, 0.0).state == "INVALID"
    assert follow_detour(points, 12.0, 0.0, math.pi).state == "INVALID"
    assert follow_detour(points[:2], 10.0, 0.0, 0.0).state == "INVALID"
    assert follow_detour(((0.0, 0.0), (1.0, 0.0), (1.2, 0.0)), 0.0, 0.0, 0.0).state == "INVALID"
    assert follow_detour(((0.0, 0.0), (math.nan, 0.0), (0.4, 0.0)), 0.0, 0.0, 0.0).state == "INVALID"


def test_detour_stanley_steers_toward_path_and_responds_to_speed():
    points = straight_candidate()
    right = follow_detour(points, 12.0, 0.20, 0.0, speed_mps=0.2)
    left = follow_detour(points, 12.0, -0.20, 0.0, speed_mps=0.2)
    faster = follow_detour(points, 12.0, 0.20, 0.0, speed_mps=1.0)
    assert right.state == left.state == faster.state == "TRACK"
    # Installed T870 convention: positive steering turns right.
    assert right.steering_rad > 0.0
    assert left.steering_rad < 0.0
    assert abs(faster.steering_rad) < abs(right.steering_rad)


def test_course_07_candidate_produces_bounded_steering_on_path():
    route_file = Path(__file__).resolve().parents[1] / "routes" / "course_07_vehicle.csv"
    route = Route.load_csv(route_file)
    x, y = route.position_at_s(210.0)
    candidate = plan_local_detour(route, 202.0, (CircleObstacle(x, y, 0.25),))
    assert candidate.status == "CANDIDATE"
    assert follow_detour(candidate.points, *route.position_at_s(202.0), route.heading_at_s(202.0)).state == "APPROACH"
    for index in (0, 5, 10, 20, 30, 40, 45):
        x, y = candidate.points[index]
        before = candidate.points[max(0, index - 1)]
        after = candidate.points[min(len(candidate.points) - 1, index + 1)]
        yaw = math.atan2(after[1] - before[1], after[0] - before[0])
        followed = follow_detour(candidate.points, x, y, yaw)
        assert followed.state == "TRACK"
        assert abs(followed.steering_rad) <= 0.30
