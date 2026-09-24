#ifndef HLKU_APP_H
#define HLKU_APP_H

#include "hlku_control.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

void hlku_app_init(const hlku_control_config_t *config);
bool hlku_app_on_udp_receive(const uint8_t *data, size_t length);
void hlku_app_tick_1khz(void);

#endif
