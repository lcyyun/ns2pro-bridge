#ifndef DS5_BRIDGE_NS2_GATT_H
#define DS5_BRIDGE_NS2_GATT_H

#include <cstddef>
#include <cstdint>

enum class Ns2GattRole : uint8_t {
    Other = 0,
    AckNotify,
    InputNotify,
    Command,
    Rumble
};

struct Ns2InitCommand {
    const char *name;
    const uint8_t *data;
    uint16_t len;
};

constexpr uint8_t NS2_UUID128_LEN = 16;

Ns2GattRole ns2_gatt_classify_uuid(uint16_t uuid16, const uint8_t uuid128[NS2_UUID128_LEN]);
bool ns2_gatt_is_input_uuid(uint16_t uuid16, const uint8_t uuid128[NS2_UUID128_LEN]);
bool ns2_gatt_uuid128_equals(const uint8_t a[NS2_UUID128_LEN], const uint8_t b[NS2_UUID128_LEN]);
const char *ns2_gatt_role_name(Ns2GattRole role);
void ns2_gatt_format_uuid(uint16_t uuid16, const uint8_t uuid128[NS2_UUID128_LEN], char *out, size_t out_len);

void ns2_gatt_set_console_mac(const uint8_t mac[6]);
uint8_t ns2_gatt_init_command_count();
const Ns2InitCommand &ns2_gatt_init_command(uint8_t index);

#endif
