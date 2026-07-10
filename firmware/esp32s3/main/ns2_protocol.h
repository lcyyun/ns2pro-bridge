#pragma once

#include <cstddef>
#include <cstdint>

namespace ns2 {

constexpr uint8_t kUsbReportIdInput = 0x05;
constexpr uint8_t kUsbReportIdOutput = 0x02;
constexpr uint8_t kUsbReportIdFeature = 0x7f;
constexpr uint16_t kNintendoVid = 0x057e;
constexpr uint16_t kNintendoPid = 0x2069;

struct AdvMatch {
    bool name = false;
    bool manufacturer = false;
    bool service_uuid = false;

    bool likely_controller() const {
        return name || manufacturer || service_uuid;
    }
};

bool name_looks_like_controller(const uint8_t *name, size_t len);
bool manufacturer_looks_pairable(const uint8_t *data, size_t len);
bool uuid128_matches_be_uuid(const uint8_t uuid_le[16], const uint8_t uuid_be[16]);
void format_ble_addr(const uint8_t addr[6], char *out, size_t out_len);

const uint8_t *fd2_notify_uuid_be();

} // namespace ns2
