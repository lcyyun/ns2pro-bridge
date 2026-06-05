#include "ns2_gatt.h"

#include <cstdio>
#include <cstring>

namespace {

const uint8_t UUID_NOTIFY_FD2[NS2_UUID128_LEN] = {
    0xab, 0x7d, 0xe9, 0xbe, 0x89, 0xfe, 0x49, 0xad,
    0x82, 0x8f, 0x11, 0x8f, 0x09, 0xdf, 0x7f, 0xd2
};

const uint8_t UUID_NOTIFY_LEGACY[NS2_UUID128_LEN] = {
    0x74, 0x92, 0x86, 0x6c, 0xec, 0x3e, 0x46, 0x19,
    0x82, 0x58, 0x32, 0x75, 0x5f, 0xfc, 0xc0, 0xf8
};

const uint8_t UUID_ACK[NS2_UUID128_LEN] = {
    0xc7, 0x65, 0xa9, 0x61, 0xd9, 0xd8, 0x4d, 0x36,
    0xa2, 0x0a, 0x53, 0x15, 0xb1, 0x11, 0x83, 0x6a
};

const uint8_t UUID_COMMAND[NS2_UUID128_LEN] = {
    0x64, 0x9d, 0x4a, 0xc9, 0x8e, 0xb7, 0x4e, 0x6c,
    0xaf, 0x44, 0x1e, 0xa5, 0x4f, 0xe5, 0xf0, 0x05
};

const uint8_t UUID_RUMBLE_CC48[NS2_UUID128_LEN] = {
    0xcc, 0x48, 0x3f, 0x51, 0x92, 0x58, 0x42, 0x7d,
    0xa9, 0x39, 0x63, 0x0c, 0x31, 0xf7, 0x2b, 0x05
};

uint8_t INIT_CMD_0[] = {0x03, 0x91, 0x01, 0x0d, 0x00, 0x08, 0x00, 0x00, 0x01, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
const uint8_t INIT_CMD_1[] = {0x07, 0x91, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00};
const uint8_t INIT_CMD_2[] = {0x16, 0x91, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00};
const uint8_t INIT_CMD_3[] = {0x15, 0x91, 0x01, 0x03, 0x00, 0x01, 0x00, 0x00, 0x00};
const uint8_t INIT_CMD_4[] = {0x0c, 0x91, 0x01, 0x02, 0x00, 0x04, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00};
const uint8_t INIT_CMD_5[] = {0x11, 0x91, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00};
const uint8_t INIT_CMD_6[] = {0x0a, 0x91, 0x01, 0x08, 0x00, 0x14, 0x00, 0x00, 0x01, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x35, 0x00, 0x46, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
const uint8_t INIT_CMD_7[] = {0x0c, 0x91, 0x01, 0x04, 0x00, 0x04, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00};
const uint8_t INIT_CMD_8[] = {0x03, 0x91, 0x01, 0x0a, 0x00, 0x04, 0x00, 0x00, 0x09, 0x00, 0x00, 0x00};
const uint8_t INIT_CMD_9[] = {0x10, 0x91, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00};
const uint8_t INIT_CMD_10[] = {0x01, 0x91, 0x01, 0x0c, 0x00, 0x00, 0x00, 0x00};
const uint8_t INIT_CMD_11[] = {0x01, 0x91, 0x01, 0x01, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
const uint8_t INIT_CMD_12[] = {0x09, 0x91, 0x01, 0x07, 0x00, 0x08, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
const uint8_t INIT_CMD_13[] = {0x02, 0x91, 0x01, 0x04, 0x00, 0x08, 0x00, 0x00, 0x09, 0x7e, 0x00, 0x00, 0xa8, 0x30, 0x01, 0x00};
const uint8_t INIT_CMD_14[] = {0x02, 0x91, 0x01, 0x04, 0x00, 0x08, 0x00, 0x00, 0x09, 0x7e, 0x00, 0x00, 0xe8, 0x30, 0x01, 0x00};

const Ns2InitCommand INIT_COMMANDS[] = {
    {"INIT", INIT_CMD_0, sizeof(INIT_CMD_0)},
    {"CMD_07", INIT_CMD_1, sizeof(INIT_CMD_1)},
    {"CMD_16", INIT_CMD_2, sizeof(INIT_CMD_2)},
    {"CMD_15_03", INIT_CMD_3, sizeof(INIT_CMD_3)},
    {"FEATSEL_SET_MASK", INIT_CMD_4, sizeof(INIT_CMD_4)},
    {"CMD_11", INIT_CMD_5, sizeof(INIT_CMD_5)},
    {"VIBRATE_CFG", INIT_CMD_6, sizeof(INIT_CMD_6)},
    {"FEATSEL_ENABLE", INIT_CMD_7, sizeof(INIT_CMD_7)},
    {"SELECT_REPORT", INIT_CMD_8, sizeof(INIT_CMD_8)},
    {"FW_INFO_GET", INIT_CMD_9, sizeof(INIT_CMD_9)},
    {"CMD_01_0C", INIT_CMD_10, sizeof(INIT_CMD_10)},
    {"RUMBLE_ENABLE", INIT_CMD_11, sizeof(INIT_CMD_11)},
    {"SET_PLAYER_LED", INIT_CMD_12, sizeof(INIT_CMD_12)},
    {"CALIB_LEFT", INIT_CMD_13, sizeof(INIT_CMD_13)},
    {"CALIB_RIGHT", INIT_CMD_14, sizeof(INIT_CMD_14)},
};

} // namespace

void ns2_gatt_set_console_mac(const uint8_t mac[6]) {
    if (!mac) {
        return;
    }
    memcpy(&INIT_CMD_0[10], mac, 6);
    printf("[NS2 GATT] console MAC in INIT set to %02x:%02x:%02x:%02x:%02x:%02x\n",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

bool ns2_gatt_uuid128_equals(const uint8_t a[NS2_UUID128_LEN], const uint8_t b[NS2_UUID128_LEN]) {
    return memcmp(a, b, NS2_UUID128_LEN) == 0;
}

Ns2GattRole ns2_gatt_classify_uuid(uint16_t uuid16, const uint8_t uuid128[NS2_UUID128_LEN]) {
    if (uuid16 != 0 || !uuid128) {
        return Ns2GattRole::Other;
    }
    if (ns2_gatt_uuid128_equals(uuid128, UUID_ACK)) {
        return Ns2GattRole::AckNotify;
    }
    if (ns2_gatt_uuid128_equals(uuid128, UUID_NOTIFY_FD2)) {
        return Ns2GattRole::InputNotify;
    }
    if (ns2_gatt_uuid128_equals(uuid128, UUID_COMMAND)) {
        return Ns2GattRole::Command;
    }
    if (ns2_gatt_uuid128_equals(uuid128, UUID_RUMBLE_CC48)) {
        return Ns2GattRole::Rumble;
    }
    return Ns2GattRole::Other;
}

bool ns2_gatt_is_input_uuid(uint16_t uuid16, const uint8_t uuid128[NS2_UUID128_LEN]) {
    return ns2_gatt_classify_uuid(uuid16, uuid128) == Ns2GattRole::InputNotify;
}

const char *ns2_gatt_role_name(Ns2GattRole role) {
    switch (role) {
        case Ns2GattRole::AckNotify:
            return "ack";
        case Ns2GattRole::InputNotify:
            return "input_notify";
        case Ns2GattRole::Command:
            return "command";
        case Ns2GattRole::Rumble:
            return "rumble";
        case Ns2GattRole::Other:
        default:
            return "other";
    }
}

void ns2_gatt_format_uuid(uint16_t uuid16, const uint8_t uuid128[NS2_UUID128_LEN], char *out, size_t out_len) {
    if (!out || out_len == 0) {
        return;
    }
    if (uuid16 != 0) {
        snprintf(out, out_len, "0x%04x", uuid16);
        return;
    }
    snprintf(out, out_len,
             "%02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x",
             uuid128[0], uuid128[1], uuid128[2], uuid128[3],
             uuid128[4], uuid128[5], uuid128[6], uuid128[7],
             uuid128[8], uuid128[9], uuid128[10], uuid128[11],
             uuid128[12], uuid128[13], uuid128[14], uuid128[15]);
}

uint8_t ns2_gatt_init_command_count() {
    return static_cast<uint8_t>(sizeof(INIT_COMMANDS) / sizeof(INIT_COMMANDS[0]));
}

const Ns2InitCommand &ns2_gatt_init_command(uint8_t index) {
    if (index >= ns2_gatt_init_command_count()) {
        index = static_cast<uint8_t>(ns2_gatt_init_command_count() - 1);
    }
    return INIT_COMMANDS[index];
}
