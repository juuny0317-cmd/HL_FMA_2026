#include "hlku_app.h"

#include "hlku_board_port.h"
#include "hlku_protocol.h"

static hlku_control_state_t control_state;
static hlku_control_output_t control_output;
static uint32_t last_feedback_ms;

void hlku_app_init(const hlku_control_config_t *config)
{
    hlku_control_init(&control_state, config);
    last_feedback_ms = hlku_board_millis();
}

bool hlku_app_on_udp_receive(const uint8_t *data, size_t length)
{
    return hlku_control_accept_packet(
        &control_state,
        data,
        (uint32_t)length,
        hlku_board_millis());
}

void hlku_app_tick_1khz(void)
{
    uint8_t buffer[sizeof(hlku_feedback_packet_t)];
    hlku_feedback_packet_t feedback;
    const uint32_t now_ms = hlku_board_millis();
    const float battery_voltage = hlku_board_read_battery_voltage();
    hlku_control_step(
        &control_state,
        now_ms,
        0.001F,
        hlku_board_read_steering_adc(),
        battery_voltage,
        hlku_board_estop_active(),
        hlku_board_overtemperature(),
        &control_output);
    hlku_board_set_drive(
        control_output.drive_pwm_01,
        control_output.drive_reverse,
        control_output.drive_brake);
    hlku_board_set_steering(
        control_output.steering_pwm_01,
        control_output.steering_reverse,
        control_output.steering_enable);
    if ((uint32_t)(now_ms - last_feedback_ms) >= 20U) {
        size_t length;
        hlku_control_make_feedback(
            &control_state, battery_voltage, &control_output, &feedback);
        length = hlku_encode_feedback(&feedback, buffer, sizeof(buffer));
        if (length > 0U) {
            hlku_board_udp_send(buffer, length);
        }
        last_feedback_ms = now_ms;
    }
}
