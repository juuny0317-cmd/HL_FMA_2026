#ifndef HLKU_PROTOCOL_H
#define HLKU_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HLKU_PROTOCOL_MAGIC UINT32_C(0x554B4C48)
#define HLKU_PROTOCOL_VERSION UINT8_C(1)
#define HLKU_PACKET_COMMAND UINT8_C(1)
#define HLKU_PACKET_FEEDBACK UINT8_C(2)

#define HLKU_FLAG_ENABLE UINT16_C(0x0001)
#define HLKU_FLAG_BRAKE UINT16_C(0x0002)

#define HLKU_FAULT_ESTOP UINT32_C(0x00000001)
#define HLKU_FAULT_WATCHDOG UINT32_C(0x00000002)
#define HLKU_FAULT_STEERING_SENSOR UINT32_C(0x00000004)
#define HLKU_FAULT_STEERING_LIMIT UINT32_C(0x00000008)
#define HLKU_FAULT_UNDERVOLTAGE UINT32_C(0x00000010)
#define HLKU_FAULT_OVERTEMPERATURE UINT32_C(0x00000020)
#define HLKU_FAULT_PROTOCOL UINT32_C(0x00000040)

#if defined(__GNUC__)
#define HLKU_PACKED __attribute__((packed))
#else
#define HLKU_PACKED
#endif

typedef struct HLKU_PACKED {
    uint32_t magic;
    uint8_t version;
    uint8_t type;
    uint16_t flags;
    uint32_t sequence;
    float drive_duty;
    float steering_angle_rad;
    uint32_t crc32;
} hlku_command_packet_t;

typedef struct HLKU_PACKED {
    uint32_t magic;
    uint8_t version;
    uint8_t type;
    uint16_t flags;
    uint32_t sequence;
    float steering_angle_rad;
    float steering_target_rad;
    float applied_drive_duty;
    float battery_voltage;
    uint32_t fault_flags;
    uint32_t crc32;
} hlku_feedback_packet_t;

uint32_t hlku_crc32(const uint8_t *data, size_t length);

bool hlku_decode_command(
    const uint8_t *data,
    size_t length,
    hlku_command_packet_t *command_out);

size_t hlku_encode_feedback(
    const hlku_feedback_packet_t *feedback,
    uint8_t *buffer,
    size_t capacity);

#ifdef __cplusplus
}
#endif

#endif
