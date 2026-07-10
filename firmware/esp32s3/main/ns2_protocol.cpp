#include "ns2_protocol.h"

#include <algorithm>
#include <cstdio>
#include <cstring>

namespace ns2 {
namespace {

constexpr uint8_t kPairableMfgPrefix[] = {0x53, 0x05, 0x01, 0x00, 0x03, 0x7e};

// Big-endian text form used by the Pico code. Advertisements carry UUID bytes
// little-endian, so uuid128_matches_be_uuid reverses before comparing.
constexpr uint8_t kFd2NotifyUuidBe[16] = {
    0xab, 0x7d, 0xe9, 0xbe, 0x89, 0xfe, 0x49, 0xad,
    0x82, 0x8f, 0x11, 0x8f, 0x09, 0xdf, 0x7f, 0xd2,
};

bool contains_ascii_ci(const uint8_t *text, size_t len, const char *needle) {
    const size_t needle_len = std::strlen(needle);
    if (needle_len == 0 || len < needle_len) {
        return false;
    }

    for (size_t i = 0; i <= len - needle_len; ++i) {
        bool match = true;
        for (size_t j = 0; j < needle_len; ++j) {
            uint8_t a = text[i + j];
            uint8_t b = static_cast<uint8_t>(needle[j]);
            if (a >= 'A' && a <= 'Z') {
                a = static_cast<uint8_t>(a + ('a' - 'A'));
            }
            if (b >= 'A' && b <= 'Z') {
                b = static_cast<uint8_t>(b + ('a' - 'A'));
            }
            if (a != b) {
                match = false;
                break;
            }
        }
        if (match) {
            return true;
        }
    }

    return false;
}

} // namespace

bool name_looks_like_controller(const uint8_t *name, size_t len) {
    if (name == nullptr || len == 0) {
        return false;
    }

    return contains_ascii_ci(name, len, "switch") ||
           contains_ascii_ci(name, len, "nintendo") ||
           contains_ascii_ci(name, len, "pro controller") ||
           contains_ascii_ci(name, len, "pro2") ||
           contains_ascii_ci(name, len, "ns2");
}

bool manufacturer_looks_pairable(const uint8_t *data, size_t len) {
    if (data == nullptr || len < sizeof(kPairableMfgPrefix)) {
        return false;
    }
    if (std::memcmp(data, kPairableMfgPrefix, sizeof(kPairableMfgPrefix)) != 0) {
        return false;
    }

    // Treat any Nintendo NS2 manufacturer prefix as a candidate on ESP32-S3.
    // The stricter "pairing advertisement" suffix check from the Pico build is
    // too narrow while we are still characterizing controller states on NimBLE.
    // Connection/GATT discovery remains the real validation gate.
    return true;
}

bool uuid128_matches_be_uuid(const uint8_t uuid_le[16], const uint8_t uuid_be[16]) {
    if (uuid_le == nullptr || uuid_be == nullptr) {
        return false;
    }

    for (size_t i = 0; i < 16; ++i) {
        if (uuid_le[i] != uuid_be[15 - i]) {
            return false;
        }
    }
    return true;
}

void format_ble_addr(const uint8_t addr[6], char *out, size_t out_len) {
    if (out == nullptr || out_len == 0) {
        return;
    }
    if (addr == nullptr) {
        std::snprintf(out, out_len, "<null>");
        return;
    }

    std::snprintf(out,
                  out_len,
                  "%02x:%02x:%02x:%02x:%02x:%02x",
                  addr[5],
                  addr[4],
                  addr[3],
                  addr[2],
                  addr[1],
                  addr[0]);
}

const uint8_t *fd2_notify_uuid_be() {
    return kFd2NotifyUuidBe;
}

} // namespace ns2
