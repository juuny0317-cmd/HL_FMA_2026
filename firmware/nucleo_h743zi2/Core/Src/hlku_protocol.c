#include "hlku_protocol.h"

#include <string.h>

_Static_assert(sizeof(hlku_command_packet_t) == 24U, "command packet size mismatch");
_Static_assert(sizeof(hlku_feedback_packet_t) == 36U, "feedback packet size mismatch");

uint32_t hlku_crc32(const uint8_t *data, size_t length)
{
    uint32_t crc = UINT32_C(0xFFFFFFFF);
    size_t index;
    for (index = 0U; index < length; ++index) {
        unsigned bit;
        crc ^= data[index];
        for (bit = 0U; bit < 8U; ++bit) {
            const uint32_t mask = (uint32_t)(-(int32_t)(crc & UINT32_C(1)));
            crc = (crc >> 1U) ^ (UINT32_C(0xEDB88320) & mask);
        }
    }
    return crc ^ UINT32_C(0xFFFFFFFF);
}

bool hlku_decode_command(
    const uint8_t *data,
    size_t length,
    hlku_command_packet_t *command_out)
{
    hlku_command_packet_t packet;
    uint32_t expected_crc;
    if ((data == NULL) || (command_out == NULL) ||
        (length != sizeof(hlku_command_packet_t))) {
        return false;
    }
    memcpy(&packet, data, sizeof(packet));
    expected_crc = hlku_crc32(data, sizeof(packet) - sizeof(packet.crc32));
    if ((packet.magic != HLKU_PROTOCOL_MAGIC) ||
        (packet.version != HLKU_PROTOCOL_VERSION) ||
        (packet.type != HLKU_PACKET_COMMAND) ||
        (packet.crc32 != expected_crc)) {
        return false;
    }
    *command_out = packet;
    return true;
}

size_t hlku_encode_feedback(
    const hlku_feedback_packet_t *feedback,
    uint8_t *buffer,
    size_t capacity)
{
    hlku_feedback_packet_t packet;
    if ((feedback == NULL) || (buffer == NULL) || (capacity < sizeof(packet))) {
        return 0U;
    }
    packet = *feedback;
    packet.magic = HLKU_PROTOCOL_MAGIC;
    packet.version = HLKU_PROTOCOL_VERSION;
    packet.type = HLKU_PACKET_FEEDBACK;
    packet.crc32 = hlku_crc32(
        (const uint8_t *)&packet,
        sizeof(packet) - sizeof(packet.crc32));
    memcpy(buffer, &packet, sizeof(packet));
    return sizeof(packet);
}
