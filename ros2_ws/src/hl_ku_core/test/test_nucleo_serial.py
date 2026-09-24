import pytest

from hl_ku_core.nucleo_serial import (
    SteeringAdcCalibration,
    drive_duty_to_command,
    drive_transaction,
    steering_transaction,
)


CALIBRATION = SteeringAdcCalibration(
    maximum_steering_rad=0.48,
    left_adc=58744,
    center_adc=31974,
    right_adc=5111,
)


def test_current_vehicle_steering_map_is_preserved():
    assert CALIBRATION.target_adc(-0.48) == 58744
    assert CALIBRATION.target_adc(0.0) == 31974
    assert CALIBRATION.target_adc(0.48) == 5111
    assert CALIBRATION.target_adc(-0.24) == 45359
    assert CALIBRATION.target_adc(0.24) == 18542


def test_route_test_drive_is_not_silently_clipped():
    assert drive_duty_to_command(0.08, 100.0, 12) == 8
    assert drive_duty_to_command(0.12, 100.0, 12) == 12
    with pytest.raises(ValueError):
        drive_duty_to_command(0.13, 100.0, 12)


def test_finger_drive_exposes_the_complete_protocol_range():
    assert drive_duty_to_command(1.0, 100.0, 100) == 100
    assert drive_duty_to_command(-1.0, 100.0, 100) == -100
    with pytest.raises(ValueError):
        drive_duty_to_command(1.01, 100.0, 100)


def test_uart_lines_match_installed_firmware_contract():
    assert drive_transaction(12) == ("CMD:12,0", "ACK CMD 12 0")
    assert steering_transaction(31974) == (
        "STEER_POS:31974",
        "ACK STEER_POS 31974",
    )
