#ifndef HLKU_BOARD_PORT_H
#define HLKU_BOARD_PORT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Implement these functions in the CubeMX-generated project. All PWM values are 0..1. */
uint32_t hlku_board_millis(void);
uint16_t hlku_board_read_steering_adc(void);
float hlku_board_read_battery_voltage(void);
bool hlku_board_estop_active(void);
bool hlku_board_overtemperature(void);
void hlku_board_set_drive(float pwm_01, bool reverse, bool brake);
void hlku_board_set_steering(float pwm_01, bool reverse, bool enable);
void hlku_board_udp_send(const uint8_t *data, size_t length);

#endif
