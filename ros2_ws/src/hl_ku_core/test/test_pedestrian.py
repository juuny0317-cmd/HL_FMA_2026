import pytest

from hl_ku_core.pedestrian import pedestrian_stop_candidates
from hl_ku_core.traffic_light import Detection


def detection(*, class_name="pedestrian", x1=300.0, y1=200.0, x2=500.0, y2=320.0):
    return Detection(
        class_name=class_name,
        confidence=0.9,
        x1=x1,
        y1=y1,
        x2=x2,
        y2=y2,
    )


def test_pedestrian_stop_candidates_require_configured_frame_height():
    accepted = detection(y1=100.0, y2=400.0)
    too_far = detection(y1=100.0, y2=399.0)

    result = pedestrian_stop_candidates(
        [accepted, too_far], 800, 600, [0.0, 0.0, 1.0, 1.0], 0.50
    )

    assert result == (accepted,)


def test_pedestrian_stop_candidates_still_apply_spatial_roi_and_class():
    centered = detection(x1=300.0, x2=500.0)
    outside = detection(x1=650.0, x2=790.0)
    wrong_class = detection(class_name="red")

    result = pedestrian_stop_candidates(
        [centered, outside, wrong_class], 800, 600, [0.25, 0.0, 0.75, 1.0], 0.20
    )

    assert result == (centered,)


@pytest.mark.parametrize("ratio", [-0.01, 1.01, float("nan")])
def test_pedestrian_stop_candidates_reject_invalid_height_ratio(ratio):
    with pytest.raises(ValueError):
        pedestrian_stop_candidates(
            [detection()], 800, 600, [0.0, 0.0, 1.0, 1.0], ratio
        )
