#pragma once

#include <cstddef>
#include <cstdint>

namespace ns2 {

constexpr uint8_t kUuid128Len = 16;

enum class GattRole : uint8_t {
    Other = 0,
    AckNotify,
    InputNotify,
    Command,
    Rumble,
};

struct InitCommand {
    const char *name;
    const uint8_t *data;
    uint16_t len;
};

GattRole gatt_classify_uuid(const uint8_t uuid128_be[kUuid128Len]);
bool gatt_uuid128_equals(const uint8_t a[kUuid128Len], const uint8_t b[kUuid128Len]);
const char *gatt_role_name(GattRole role);
void gatt_format_uuid(const uint8_t uuid128_be[kUuid128Len], char *out, size_t out_len);
void gatt_set_console_mac(const uint8_t mac[6]);
uint8_t gatt_init_command_count();
const InitCommand &gatt_init_command(uint8_t index);
const uint8_t *gatt_fd2_uuid();
const uint8_t *gatt_legacy_uuid();

} // namespace ns2
