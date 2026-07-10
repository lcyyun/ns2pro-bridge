#pragma once

#include <cstddef>
#include <cstdint>

namespace ns2 {

constexpr size_t kMotionSampleSize = 12;
constexpr size_t kInputRawReportMax = 64;

enum class InputReportKind : uint8_t {
    Unknown = 0,
    Fd2,
    Legacy,
};

struct InputSnapshot {
    bool valid = false;
    InputReportKind kind = InputReportKind::Unknown;
    uint16_t len = 0;
    uint8_t first_byte = 0;
    uint32_t updates = 0;
    uint32_t parse_errors = 0;
    uint32_t buttons = 0;
    uint16_t lx = 2048;
    uint16_t ly = 2048;
    uint16_t rx = 2048;
    uint16_t ry = 2048;
    bool battery_valid = false;
    uint8_t battery_raw = 0;
    uint8_t battery_percent = 0;
    bool battery_charging = false;
    uint8_t battery_offset = 0xff;
    bool raw_valid = false;
    uint16_t raw_len = 0;
    uint8_t raw[kInputRawReportMax] = {};
    bool motion_valid = false;
    uint32_t motion_updates = 0;
    InputReportKind motion_kind = InputReportKind::Unknown;
    uint16_t motion_len = 0;
    uint8_t motion_offset = 0;
    uint8_t motion[kMotionSampleSize] = {};
};

void input_reset();
bool input_parse_notify(InputReportKind kind, const uint8_t *data, uint16_t len);
bool input_get_snapshot(InputSnapshot *out);
uint32_t input_updates();
uint32_t input_parse_errors();
uint32_t input_last_age_ms();
const char *input_kind_name(InputReportKind kind);
void input_format_motion_json(const InputSnapshot *snapshot, char *out, size_t out_len);

} // namespace ns2
