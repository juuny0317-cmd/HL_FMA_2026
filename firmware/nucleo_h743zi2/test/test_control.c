#include "hlku_control.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static hlku_control_config_t config(void)
{
    const hlku_control_config_t value = {
        .steering_adc_left = 1000U,
        .steering_adc_center = 2000U,
        .steering_adc_right = 3000U,
        .steering_left_rad = -0.5F,
        .steering_right_rad = 0.5F,
        .steering_kp = 1.0F,
        .steering_ki = 0.0F,
        .steering_kd = 0.0F,
        .steering_integral_limit = 0.5F,
        .steering_output_limit = 0.7F,
        .steering_deadband_rad = 0.01F,
        .drive_duty_limit = 0.6F,
        .drive_slew_per_sec = 10.0F,
        .minimum_battery_voltage = 18.0F,
        .watchdog_timeout_ms = 100U,
    };
    return value;
}

static size_t command_packet(
    uint8_t *buffer,
    uint16_t flags,
    uint32_t sequence,
    float duty,
    float steering)
{
    hlku_command_packet_t packet;
    memset(&packet, 0, sizeof(packet));
    packet.magic = HLKU_PROTOCOL_MAGIC;
    packet.version = HLKU_PROTOCOL_VERSION;
    packet.type = HLKU_PACKET_COMMAND;
    packet.flags = flags;
    packet.sequence = sequence;
    packet.drive_duty = duty;
    packet.steering_angle_rad = steering;
    packet.crc32 = hlku_crc32(
        (const uint8_t *)&packet,
        sizeof(packet) - sizeof(packet.crc32));
    memcpy(buffer, &packet, sizeof(packet));
    return sizeof(packet);
}

static void test_watchdog_and_valid_drive(void)
{
    uint8_t packet[sizeof(hlku_command_packet_t)];
    hlku_control_state_t state;
    hlku_control_output_t output;
    const hlku_control_config_t cfg = config();
    hlku_control_init(&state, &cfg);
    hlku_control_step(&state, 0U, 0.001F, 2000U, 22.0F, false, false, &output);
    assert((state.fault_flags & HLKU_FAULT_WATCHDOG) != 0U);
    assert(output.drive_brake);
    assert(!output.system_enabled);

    assert(hlku_control_accept_packet(
        &state,
        packet,
        (uint32_t)command_packet(
            packet, HLKU_FLAG_ENABLE, 1U, 0.3F, 0.1F),
        10U));
    hlku_control_step(&state, 10U, 0.001F, 2000U, 22.0F, false, false, &output);
    assert(state.fault_flags == 0U);
    assert(output.system_enabled);
    assert(output.drive_pwm_01 > 0.0F);
    assert(!output.drive_brake);

    hlku_control_step(&state, 111U, 0.001F, 2000U, 22.0F, false, false, &output);
    assert((state.fault_flags & HLKU_FAULT_WATCHDOG) != 0U);
    assert(output.drive_brake);
}

static void test_invalid_flags_and_battery_fail_closed(void)
{
    uint8_t packet[sizeof(hlku_command_packet_t)];
    hlku_control_state_t state;
    hlku_control_output_t output;
    const hlku_control_config_t cfg = config();
    hlku_control_init(&state, &cfg);
    assert(!hlku_control_accept_packet(
        &state,
        packet,
        (uint32_t)command_packet(
            packet,
            HLKU_FLAG_ENABLE | HLKU_FLAG_BRAKE,
            1U,
            0.0F,
            0.0F),
        0U));
    assert((state.fault_flags & HLKU_FAULT_PROTOCOL) != 0U);

    assert(hlku_control_accept_packet(
        &state,
        packet,
        (uint32_t)command_packet(
            packet, HLKU_FLAG_ENABLE, 2U, 0.2F, 0.0F),
        1U));
    hlku_control_step(&state, 1U, 0.001F, 2000U, 0.0F, false, false, &output);
    assert((state.fault_flags & HLKU_FAULT_UNDERVOLTAGE) != 0U);
    assert(!output.system_enabled);
    assert(output.drive_brake);
}

static void test_estop_latches_until_reset(void)
{
    uint8_t packet[sizeof(hlku_command_packet_t)];
    hlku_control_state_t state;
    hlku_control_output_t output;
    const hlku_control_config_t cfg = config();
    hlku_control_init(&state, &cfg);
    assert(hlku_control_accept_packet(
        &state,
        packet,
        (uint32_t)command_packet(
            packet, HLKU_FLAG_ENABLE, 1U, 0.2F, 0.0F),
        0U));
    hlku_control_step(&state, 0U, 0.001F, 2000U, 22.0F, true, false, &output);
    assert((state.fault_flags & HLKU_FAULT_ESTOP) != 0U);
    hlku_control_step(&state, 1U, 0.001F, 2000U, 22.0F, false, false, &output);
    assert((state.fault_flags & HLKU_FAULT_ESTOP) != 0U);
    assert(output.drive_brake);
}

int main(void)
{
    test_watchdog_and_valid_drive();
    test_invalid_flags_and_battery_fail_closed();
    test_estop_latches_until_reset();
    puts("firmware host tests passed");
    return 0;
}
