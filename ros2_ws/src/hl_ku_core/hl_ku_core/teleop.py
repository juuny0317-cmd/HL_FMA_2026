"""Pure helpers shared by interactive vehicle teleoperation modes."""

from __future__ import annotations

import math


def fixed_steering_target(direction: int, maximum_steering_rad: float) -> float:
    """Return one fixed left/right steering target; repeated keys are idempotent."""

    if direction not in (-1, 1):
        raise ValueError("direction must be -1 or 1")
    if not math.isfinite(maximum_steering_rad) or maximum_steering_rad <= 0.0:
        raise ValueError("maximum steering must be finite and positive")
    return direction * maximum_steering_rad


def step_steering_target(
    current: float,
    direction: int,
    step_rad: float,
    maximum_steering_rad: float,
) -> float:
    """Move one steering step and retain that target until the next key."""

    if direction not in (-1, 1):
        raise ValueError("direction must be -1 or 1")
    if not math.isfinite(step_rad) or step_rad <= 0.0:
        raise ValueError("steering step must be finite and positive")
    if not math.isfinite(maximum_steering_rad) or maximum_steering_rad <= 0.0:
        raise ValueError("maximum steering must be finite and positive")
    return max(
        -maximum_steering_rad,
        min(maximum_steering_rad, current + direction * step_rad),
    )


def manual_control_active(
    emergency_latched: bool,
    moving: bool,
    persistent_steering_active: bool,
    timed_steering_active: bool,
) -> bool:
    """Keep a persistent steering target active after an A/D/C key press."""

    return not emergency_latched and (
        moving or persistent_steering_active or timed_steering_active
    )


def direct_signed_drive(
    direction: int,
    selected_magnitude: float,
    maximum_magnitude: float,
) -> float:
    """Return the selected full-range command for W (+) or S (-)."""

    if direction not in (-1, 1):
        raise ValueError("direction must be -1 or 1")
    if maximum_magnitude <= 0.0:
        raise ValueError("maximum magnitude must be positive")
    if not 0.0 <= selected_magnitude <= maximum_magnitude:
        raise ValueError("selected drive magnitude is outside the configured range")
    return direction * selected_magnitude


def three_state_signed_drive(
    current: float,
    direction: int,
    selected_magnitude: float,
    maximum_magnitude: float,
) -> float:
    """Move between reverse, stop and forward without skipping stop."""

    if direction not in (-1, 1):
        raise ValueError("direction must be -1 or 1")
    if maximum_magnitude <= 0.0:
        raise ValueError("maximum magnitude must be positive")
    if not 0.0 <= selected_magnitude <= maximum_magnitude:
        raise ValueError("selected drive magnitude is outside the configured range")
    if direction > 0:
        return 0.0 if current < -1.0e-9 else selected_magnitude
    return 0.0 if current > 1.0e-9 else -selected_magnitude


def step_signed_drive(
    current: float,
    direction: int,
    initial_magnitude: float,
    step: float,
    maximum_magnitude: float,
) -> float:
    """Step a persistent signed command; W=+1 and S=-1."""

    if direction not in (-1, 1):
        raise ValueError("direction must be -1 or 1")
    if not 0.0 <= initial_magnitude <= maximum_magnitude:
        raise ValueError("initial drive magnitude is outside the configured range")
    if step <= 0.0 or maximum_magnitude <= 0.0:
        raise ValueError("drive step and maximum magnitude must be positive")
    if abs(current) < 1.0e-9:
        return direction * initial_magnitude
    return max(
        -maximum_magnitude,
        min(maximum_magnitude, current + direction * step),
    )


def manual_command_status(
    key: str,
    drive_duty: float,
    configured_duty: float,
    steering_rad: float,
    brake: bool,
    enabled: bool,
    fault: bool,
) -> str:
    """Build the one-line live status shown during finger driving."""
    if drive_duty > 1.0e-9:
        drive = f"전진 +{round(drive_duty * 100):d}/100"
    elif drive_duty < -1.0e-9:
        drive = f"후진 {round(drive_duty * 100):d}/100"
    else:
        drive = "정지 0/100"

    if steering_rad < -1.0e-9:
        steering = "좌"
    elif steering_rad > 1.0e-9:
        steering = "우"
    else:
        steering = "중앙"
    steering += f" {steering_rad:+.3f} rad ({math.degrees(steering_rad):+.1f}°)"

    if fault:
        state = "FAULT"
    elif brake:
        state = "브레이크"
    elif enabled and abs(drive_duty) <= 1.0e-9:
        state = "조향만 출력 중 · W/S 구동 가능"
    elif enabled:
        state = "출력 중"
    else:
        state = "대기"
    return (
        f"입력={key or '-'} | 구동={drive} | 조향={steering} | "
        f"PWM단위={round(configured_duty * 100):d}/100 | 상태={state}"
    )
