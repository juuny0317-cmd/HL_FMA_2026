#ifndef HLKU_CONTROL_H
#define HLKU_CONTROL_H

#include "hlku_protocol.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint16_t steering_adc_left;
    uint16_t steering_adc_center;
    uint16_t steering_adc_right;
    float steering_left_rad;
    float steering_right_rad;
    float steering_kp;
    float steering_ki;
    float steering_kd;
    float steering_integral_limit;
    float steering_output_limit;
    float steering_deadband_rad;
    float drive_duty_limit;
    float drive_slew_per_sec;
    float minimum_battery_voltage;
    uint32_t watchdog_timeout_ms;
} hlku_control_config_t;

typedef struct {
    hlku_control_config_t config;
    hlku_command_packet_t command;
    uint32_t last_command_ms;
    uint32_t fault_flags;
    float steering_integral;
    float previous_steering_error;
    float steering_angle_rad;
    float steering_output;
    float applied_drive_duty;
    bool command_received;
} hlku_control_state_t;

typedef struct {
    float drive_pwm_01;
    bool drive_reverse;
    bool drive_brake;
    float steering_pwm_01;
    bool steering_reverse;
    bool steering_enable;
    bool system_enabled;
} hlku_control_output_t;

void hlku_control_init(
    hlku_control_state_t *state,
    const hlku_control_config_t *config);

bool hlku_control_accept_packet(
    hlku_control_state_t *state,
    const uint8_t *data,
    uint32_t length,
    uint32_t now_ms);

void hlku_control_step(
    hlku_control_state_t *state,
    uint32_t now_ms,
    float dt_sec,
    uint16_t steering_adc,
    float battery_voltage,
    bool emergency_stop_active,
    bool overtemperature,
    hlku_control_output_t *output);

void hlku_control_make_feedback(
    const hlku_control_state_t *state,
    float battery_voltage,
    const hlku_control_output_t *output,
    hlku_feedback_packet_t *feedback);

#ifdef __cplusplus
}
#endif

#endif
