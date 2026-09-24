"""Pure helpers for the currently installed NUCLEO UART command protocol."""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class SteeringAdcCalibration:
    """Piecewise-linear steering map used by the existing vehicle firmware.

    The vehicle convention is negative steering = left.  Its potentiometer ADC
    increases toward the left, so the ordering is right < center < left.
    """

    maximum_steering_rad: float
    left_adc: int
    center_adc: int
    right_adc: int

    def validate(self) -> None:
        if not math.isfinite(self.maximum_steering_rad) or self.maximum_steering_rad <= 0.0:
            raise ValueError("maximum_steering_rad must be positive and finite")
        if not (0 <= self.right_adc < self.center_adc < self.left_adc <= 65535):
            raise ValueError("steering ADC order must be right < center < left")

    def target_adc(self, steering_angle_rad: float) -> int:
        self.validate()
        if not math.isfinite(steering_angle_rad):
            raise ValueError("steering angle must be finite")
        ratio = max(
            -1.0,
            min(1.0, steering_angle_rad / self.maximum_steering_rad),
        )
        if ratio < 0.0:
            return round(
                self.center_adc + (-ratio) * (self.left_adc - self.center_adc)
            )
        return round(
            self.center_adc + ratio * (self.right_adc - self.center_adc)
        )


def drive_duty_to_command(
    drive_duty: float,
    command_scale: float,
    maximum_command: int,
) -> int:
    """Convert normalized drive duty without silently clipping an unsafe input."""

    if not all(math.isfinite(value) for value in (drive_duty, command_scale)):
        raise ValueError("drive conversion values must be finite")
    if command_scale <= 0.0 or maximum_command <= 0:
        raise ValueError("drive command limits must be positive")
    command = round(drive_duty * command_scale)
    if abs(command) > maximum_command:
        raise ValueError("drive duty exceeds the NUCLEO route-test limit")
    return command


def drive_transaction(command: int) -> tuple[str, str]:
    return f"CMD:{command},0", f"ACK CMD {command} 0"


def steering_transaction(target_adc: int) -> tuple[str, str]:
    return f"STEER_POS:{target_adc}", f"ACK STEER_POS {target_adc}"
