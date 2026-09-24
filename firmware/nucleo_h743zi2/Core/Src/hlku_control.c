#include "hlku_control.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static float clampf(float value, float lower, float upper)
{
    if (value < lower) {
        return lower;
    }
    if (value > upper) {
        return upper;
    }
    return value;
}

static float approach(float current, float target, float maximum_change)
{
    const float delta = clampf(target - current, -maximum_change, maximum_change);
    return current + delta;
}

static bool steering_from_adc(
    const hlku_control_config_t *config,
    uint16_t adc,
    float *angle_out)
{
    const uint16_t low = config->steering_adc_left < config->steering_adc_right
        ? config->steering_adc_left : config->steering_adc_right;
    const uint16_t high = config->steering_adc_left > config->steering_adc_right
        ? config->steering_adc_left : config->steering_adc_right;
    float angle;
    if ((angle_out == NULL) || (config->steering_adc_center <= low) ||
        (config->steering_adc_center >= high) || (adc + 80U < low) ||
        (adc > high + 80U)) {
        return false;
    }
    if (adc <= config->steering_adc_center) {
        const float ratio = (float)(adc - low) /
            (float)(config->steering_adc_center - low);
        const float low_angle = config->steering_adc_left == low
            ? config->steering_left_rad : config->steering_right_rad;
        angle = low_angle + ratio * (0.0F - low_angle);
    } else {
        const float ratio = (float)(adc - config->steering_adc_center) /
            (float)(high - config->steering_adc_center);
        const float high_angle = config->steering_adc_right == high
            ? config->steering_right_rad : config->steering_left_rad;
        angle = ratio * high_angle;
    }
    *angle_out = angle;
    return isfinite(angle);
}

void hlku_control_init(
    hlku_control_state_t *state,
    const hlku_control_config_t *config)
{
    if ((state == NULL) || (config == NULL)) {
        return;
    }
    memset(state, 0, sizeof(*state));
    state->config = *config;
    state->command.magic = HLKU_PROTOCOL_MAGIC;
    state->command.version = HLKU_PROTOCOL_VERSION;
    state->command.type = HLKU_PACKET_COMMAND;
    state->command.flags = HLKU_FLAG_BRAKE;
}

bool hlku_control_accept_packet(
    hlku_control_state_t *state,
    const uint8_t *data,
    uint32_t length,
    uint32_t now_ms)
{
    hlku_command_packet_t command;
    if (state == NULL) {
        return false;
    }
    if (!hlku_decode_command(data, length, &command) ||
        !isfinite(command.drive_duty) ||
        !isfinite(command.steering_angle_rad) ||
        (fabsf(command.drive_duty) > 1.0F) ||
        ((command.flags & ~(HLKU_FLAG_ENABLE | HLKU_FLAG_BRAKE)) != 0U) ||
        (((command.flags & HLKU_FLAG_ENABLE) != 0U) ==
         ((command.flags & HLKU_FLAG_BRAKE) != 0U)) ||
        (((command.flags & HLKU_FLAG_BRAKE) != 0U) &&
         (fabsf(command.drive_duty) > 0.0001F))) {
        state->fault_flags |= HLKU_FAULT_PROTOCOL;
        return false;
    }
    state->fault_flags &= ~HLKU_FAULT_PROTOCOL;
    state->command = command;
    state->last_command_ms = now_ms;
    state->command_received = true;
    return true;
}

void hlku_control_step(
    hlku_control_state_t *state,
    uint32_t now_ms,
    float dt_sec,
    uint16_t steering_adc,
    float battery_voltage,
    bool emergency_stop_active,
    bool overtemperature,
    hlku_control_output_t *output)
{
    bool watchdog_ok;
    bool steering_ok;
    bool enabled;
    float target_angle;
    float target_duty;
    float error;
    float derivative;
    float steering_command;
    if ((state == NULL) || (output == NULL)) {
        return;
    }
    memset(output, 0, sizeof(*output));
    watchdog_ok = state->command_received &&
        ((uint32_t)(now_ms - state->last_command_ms) <= state->config.watchdog_timeout_ms);
    if (watchdog_ok) {
        state->fault_flags &= ~HLKU_FAULT_WATCHDOG;
    } else {
        state->fault_flags |= HLKU_FAULT_WATCHDOG;
    }
    if (emergency_stop_active) {
        state->fault_flags |= HLKU_FAULT_ESTOP;
    }
    if (overtemperature) {
        state->fault_flags |= HLKU_FAULT_OVERTEMPERATURE;
    } else {
        state->fault_flags &= ~HLKU_FAULT_OVERTEMPERATURE;
    }
    if (!isfinite(battery_voltage) ||
        battery_voltage < state->config.minimum_battery_voltage) {
        state->fault_flags |= HLKU_FAULT_UNDERVOLTAGE;
    } else {
        state->fault_flags &= ~HLKU_FAULT_UNDERVOLTAGE;
    }
    steering_ok = steering_from_adc(
        &state->config, steering_adc, &state->steering_angle_rad);
    if (steering_ok) {
        state->fault_flags &= ~HLKU_FAULT_STEERING_SENSOR;
    } else {
        state->fault_flags |= HLKU_FAULT_STEERING_SENSOR;
    }
    if ((state->command.steering_angle_rad < state->config.steering_left_rad - 0.05F) ||
        (state->command.steering_angle_rad > state->config.steering_right_rad + 0.05F)) {
        state->fault_flags |= HLKU_FAULT_STEERING_LIMIT;
    } else {
        state->fault_flags &= ~HLKU_FAULT_STEERING_LIMIT;
    }
    enabled = watchdog_ok && (state->fault_flags == 0U) &&
        ((state->command.flags & HLKU_FLAG_ENABLE) != 0U) &&
        ((state->command.flags & HLKU_FLAG_BRAKE) == 0U);
    output->system_enabled = enabled;
    if (!enabled) {
        state->steering_integral = 0.0F;
        state->steering_output = 0.0F;
        state->applied_drive_duty = 0.0F;
        output->drive_brake = true;
        return;
    }
    target_angle = clampf(
        state->command.steering_angle_rad,
        state->config.steering_left_rad,
        state->config.steering_right_rad);
    error = target_angle - state->steering_angle_rad;
    if (fabsf(error) <= state->config.steering_deadband_rad) {
        error = 0.0F;
    }
    if (dt_sec > 0.0001F && dt_sec <= 0.1F) {
        state->steering_integral = clampf(
            state->steering_integral + error * dt_sec,
            -state->config.steering_integral_limit,
            state->config.steering_integral_limit);
        derivative = (error - state->previous_steering_error) / dt_sec;
    } else {
        derivative = 0.0F;
    }
    state->previous_steering_error = error;
    steering_command = state->config.steering_kp * error +
        state->config.steering_ki * state->steering_integral +
        state->config.steering_kd * derivative;
    state->steering_output = clampf(
        steering_command,
        -state->config.steering_output_limit,
        state->config.steering_output_limit);
    output->steering_enable = true;
    output->steering_reverse = state->steering_output < 0.0F;
    output->steering_pwm_01 = fabsf(state->steering_output);
    target_duty = clampf(
        state->command.drive_duty,
        -state->config.drive_duty_limit,
        state->config.drive_duty_limit);
    state->applied_drive_duty = approach(
        state->applied_drive_duty,
        target_duty,
        fmaxf(0.0F, state->config.drive_slew_per_sec * dt_sec));
    output->drive_reverse = state->applied_drive_duty < 0.0F;
    output->drive_pwm_01 = fabsf(state->applied_drive_duty);
    output->drive_brake = output->drive_pwm_01 <= 0.0001F;
}

void hlku_control_make_feedback(
    const hlku_control_state_t *state,
    float battery_voltage,
    const hlku_control_output_t *output,
    hlku_feedback_packet_t *feedback)
{
    if ((state == NULL) || (output == NULL) || (feedback == NULL)) {
        return;
    }
    memset(feedback, 0, sizeof(*feedback));
    feedback->flags = (output->system_enabled ? HLKU_FLAG_ENABLE : 0U) |
        (output->drive_brake ? HLKU_FLAG_BRAKE : 0U);
    feedback->sequence = state->command.sequence;
    feedback->steering_angle_rad = state->steering_angle_rad;
    feedback->steering_target_rad = state->command.steering_angle_rad;
    feedback->applied_drive_duty = state->applied_drive_duty;
    feedback->battery_voltage = battery_voltage;
    feedback->fault_flags = state->fault_flags;
}
