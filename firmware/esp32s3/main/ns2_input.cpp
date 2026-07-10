#include "ns2_input.h"

#include <algorithm>
#include <cstdio>
#include <cstring>

#include "esp_timer.h"

namespace ns2 {
namespace {

constexpr uint16_t kCenter12 = 2048;
constexpr int32_t kAxisDeadzone = 48;
constexpr uint32_t kAxisCalibrationSamples = 20;
constexpr uint16_t kFd2FullReportMinLen = 60;
constexpr uint16_t kFd2FullMotionOffset = 48;
constexpr uint32_t kFd2KnownButtonMask =
    0x00000001u | 0x00000002u | 0x00000004u | 0x00000008u |
    0x00000040u | 0x00000080u |
    0x00000100u | 0x00000200u | 0x00000400u | 0x00000800u |
    0x00001000u | 0x00002000u | 0x00004000u |
    0x00010000u | 0x00020000u | 0x00040000u | 0x00080000u |
    0x00400000u | 0x00800000u | 0x01000000u | 0x02000000u;

enum Button : uint8_t {
    ButtonB = 0,
    ButtonA,
    ButtonY,
    ButtonX,
    ButtonR,
    ButtonZR,
    ButtonPlus,
    ButtonRStick,
    ButtonDDown,
    ButtonDRight,
    ButtonDLeft,
    ButtonDUp,
    ButtonL,
    ButtonZL,
    ButtonMinus,
    ButtonLStick,
    ButtonHome,
    ButtonCapture,
    ButtonGR,
    ButtonGL,
    ButtonC,
    ButtonCount,
};

struct AxisCalibration {
    bool calibrated = false;
    uint32_t sample_count = 0;
    uint32_t sum_lx = 0;
    uint32_t sum_ly = 0;
    uint32_t sum_rx = 0;
    uint32_t sum_ry = 0;
    uint16_t center_lx = kCenter12;
    uint16_t center_ly = kCenter12;
    uint16_t center_rx = kCenter12;
    uint16_t center_ry = kCenter12;
};

InputSnapshot s_input;
AxisCalibration s_fd2_axis;
AxisCalibration s_legacy_axis;
uint64_t s_last_input_us = 0;

void reset_axis(AxisCalibration *axis) {
    *axis = AxisCalibration{};
}

uint32_t read_le32(const uint8_t *p) {
    return static_cast<uint32_t>(p[0]) |
           (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) |
           (static_cast<uint32_t>(p[3]) << 24);
}

int16_t read_le16s(const uint8_t *p) {
    return static_cast<int16_t>(static_cast<uint16_t>(p[0]) |
                                (static_cast<uint16_t>(p[1]) << 8));
}

uint16_t clamp12(int32_t value) {
    if (value < 0) {
        return 0;
    }
    if (value > 4095) {
        return 4095;
    }
    return static_cast<uint16_t>(value);
}

uint16_t unpack12_x(const uint8_t *data, int offset) {
    return clamp12(static_cast<int32_t>(data[offset]) |
                   ((static_cast<int32_t>(data[offset + 1]) & 0x0f) << 8));
}

uint16_t unpack12_y(const uint8_t *data, int offset) {
    return clamp12(((static_cast<int32_t>(data[offset + 1]) >> 4) & 0x0f) |
                   (static_cast<int32_t>(data[offset + 2]) << 4));
}

uint16_t recenter_axis(uint16_t value, uint16_t center) {
    const uint16_t clamped = clamp12(static_cast<int32_t>(kCenter12) +
                                     static_cast<int32_t>(value) -
                                     static_cast<int32_t>(center));
    int32_t delta = static_cast<int32_t>(clamped) - kCenter12;
    if (delta < 0) {
        delta = -delta;
    }
    return delta <= kAxisDeadzone ? kCenter12 : clamped;
}

void set_button(uint32_t *buttons, Button button, bool pressed) {
    if (button >= ButtonCount) {
        return;
    }
    const uint32_t mask = 1u << static_cast<uint8_t>(button);
    if (pressed) {
        *buttons |= mask;
    } else {
        *buttons &= ~mask;
    }
}

void clear_battery(InputSnapshot *out) {
    out->battery_valid = false;
    out->battery_raw = 0;
    out->battery_percent = 0;
    out->battery_charging = false;
    out->battery_offset = 0xff;
}

void apply_switch_battery_guess(InputSnapshot *out, const uint8_t *data, uint16_t len, uint8_t offset) {
    if (offset >= len) {
        return;
    }

    const uint8_t raw = data[offset];
    const uint8_t level = (raw >> 4) & 0x0f;
    out->battery_raw = raw;
    out->battery_offset = offset;
    out->battery_charging = (raw & 0x01) != 0;

    // Switch-style reports usually carry battery as a high-nibble 0..8 or 0..10 level.
    // Keep the raw byte visible even when the heuristic does not look plausible.
    if (raw != 0x00 && raw != 0xff && level <= 10) {
        const uint8_t percent = static_cast<uint8_t>(level * 10);
        out->battery_valid = true;
        out->battery_percent = percent > 100 ? 100 : percent;
    }
}

uint32_t decode_legacy_buttons(uint8_t b2, uint8_t b3, uint8_t b4) {
    uint32_t buttons = 0;
    set_button(&buttons, ButtonB, (b2 & 0x01) != 0);
    set_button(&buttons, ButtonA, (b2 & 0x02) != 0);
    set_button(&buttons, ButtonY, (b2 & 0x04) != 0);
    set_button(&buttons, ButtonX, (b2 & 0x08) != 0);
    set_button(&buttons, ButtonR, (b2 & 0x10) != 0);
    set_button(&buttons, ButtonZR, (b2 & 0x20) != 0);
    set_button(&buttons, ButtonPlus, (b2 & 0x40) != 0);
    set_button(&buttons, ButtonRStick, (b2 & 0x80) != 0);
    set_button(&buttons, ButtonDDown, (b3 & 0x01) != 0);
    set_button(&buttons, ButtonDRight, (b3 & 0x02) != 0);
    set_button(&buttons, ButtonDLeft, (b3 & 0x04) != 0);
    set_button(&buttons, ButtonDUp, (b3 & 0x08) != 0);
    set_button(&buttons, ButtonL, (b3 & 0x10) != 0);
    set_button(&buttons, ButtonZL, (b3 & 0x20) != 0);
    set_button(&buttons, ButtonMinus, (b3 & 0x40) != 0);
    set_button(&buttons, ButtonLStick, (b3 & 0x80) != 0);
    set_button(&buttons, ButtonHome, (b4 & 0x01) != 0);
    set_button(&buttons, ButtonCapture, (b4 & 0x02) != 0);
    set_button(&buttons, ButtonGR, (b4 & 0x04) != 0);
    set_button(&buttons, ButtonGL, (b4 & 0x08) != 0);
    set_button(&buttons, ButtonC, (b4 & 0x10) != 0);
    return buttons;
}

uint32_t decode_fd2_buttons(uint32_t raw) {
    uint32_t buttons = 0;
    set_button(&buttons, ButtonY, (raw & 0x00000001) != 0);
    set_button(&buttons, ButtonX, (raw & 0x00000002) != 0);
    set_button(&buttons, ButtonB, (raw & 0x00000004) != 0);
    set_button(&buttons, ButtonA, (raw & 0x00000008) != 0);
    set_button(&buttons, ButtonR, (raw & 0x00000040) != 0);
    set_button(&buttons, ButtonZR, (raw & 0x00000080) != 0);
    set_button(&buttons, ButtonMinus, (raw & 0x00000100) != 0);
    set_button(&buttons, ButtonPlus, (raw & 0x00000200) != 0);
    set_button(&buttons, ButtonRStick, (raw & 0x00000400) != 0);
    set_button(&buttons, ButtonLStick, (raw & 0x00000800) != 0);
    set_button(&buttons, ButtonHome, (raw & 0x00001000) != 0);
    set_button(&buttons, ButtonCapture, (raw & 0x00002000) != 0);
    set_button(&buttons, ButtonC, (raw & 0x00004000) != 0);
    set_button(&buttons, ButtonDDown, (raw & 0x00010000) != 0);
    set_button(&buttons, ButtonDUp, (raw & 0x00020000) != 0);
    set_button(&buttons, ButtonDRight, (raw & 0x00040000) != 0);
    set_button(&buttons, ButtonDLeft, (raw & 0x00080000) != 0);
    set_button(&buttons, ButtonL, (raw & 0x00400000) != 0);
    set_button(&buttons, ButtonZL, (raw & 0x00800000) != 0);
    set_button(&buttons, ButtonGR, (raw & 0x01000000) != 0);
    set_button(&buttons, ButtonGL, (raw & 0x02000000) != 0);
    return buttons;
}

void apply_axes(AxisCalibration *axis, InputSnapshot *state, uint16_t lx, uint16_t ly, uint16_t rx, uint16_t ry) {
    if (!axis->calibrated && state->buttons == 0) {
        axis->sum_lx += lx;
        axis->sum_ly += ly;
        axis->sum_rx += rx;
        axis->sum_ry += ry;
        axis->sample_count++;
        if (axis->sample_count >= kAxisCalibrationSamples) {
            axis->center_lx = static_cast<uint16_t>(axis->sum_lx / axis->sample_count);
            axis->center_ly = static_cast<uint16_t>(axis->sum_ly / axis->sample_count);
            axis->center_rx = static_cast<uint16_t>(axis->sum_rx / axis->sample_count);
            axis->center_ry = static_cast<uint16_t>(axis->sum_ry / axis->sample_count);
            axis->calibrated = true;
        }
    }

    if (!axis->calibrated) {
        state->lx = kCenter12;
        state->ly = kCenter12;
        state->rx = kCenter12;
        state->ry = kCenter12;
        return;
    }
    state->lx = recenter_axis(lx, axis->center_lx);
    state->ly = recenter_axis(ly, axis->center_ly);
    state->rx = recenter_axis(rx, axis->center_rx);
    state->ry = recenter_axis(ry, axis->center_ry);
}

bool parse_fd2(const uint8_t *data, uint16_t len, InputSnapshot *out) {
    if (len < 16) {
        return false;
    }
    const uint32_t raw_buttons = read_le32(data + 4);
    if ((raw_buttons & ~kFd2KnownButtonMask) != 0) {
        return false;
    }
    out->buttons = decode_fd2_buttons(raw_buttons);
    apply_switch_battery_guess(out, data, len, 3);
    apply_axes(&s_fd2_axis, out, unpack12_x(data, 10), unpack12_y(data, 10), unpack12_x(data, 13), unpack12_y(data, 13));
    if (len >= kFd2FullReportMinLen) {
        std::memcpy(out->motion, data + kFd2FullMotionOffset, kMotionSampleSize);
        out->motion_valid = true;
        out->motion_updates++;
        out->motion_kind = InputReportKind::Fd2;
        out->motion_len = len;
        out->motion_offset = kFd2FullMotionOffset;
    }
    return true;
}

bool parse_legacy(const uint8_t *data, uint16_t len, InputSnapshot *out) {
    if (len < 11) {
        return false;
    }
    if ((data[4] & 0xe0) != 0) {
        return false;
    }
    clear_battery(out);
    out->buttons = decode_legacy_buttons(data[2], data[3], data[4]);
    apply_axes(&s_legacy_axis, out, unpack12_x(data, 5), unpack12_y(data, 5), unpack12_x(data, 8), unpack12_y(data, 8));
    return true;
}

} // namespace

void input_reset() {
    s_input = InputSnapshot{};
    reset_axis(&s_fd2_axis);
    reset_axis(&s_legacy_axis);
    s_last_input_us = 0;
}

bool input_parse_notify(InputReportKind kind, const uint8_t *data, uint16_t len) {
    if (data == nullptr || len == 0) {
        s_input.parse_errors++;
        return false;
    }

    InputSnapshot next = s_input;
    next.valid = false;
    next.kind = kind;
    next.len = len;
    next.first_byte = data[0];
    next.raw_valid = true;
    next.raw_len = len > kInputRawReportMax ? kInputRawReportMax : len;
    std::memcpy(next.raw, data, next.raw_len);
    if (next.raw_len < sizeof(next.raw)) {
        std::memset(next.raw + next.raw_len, 0, sizeof(next.raw) - next.raw_len);
    }
    next.motion_valid = false;
    next.motion_kind = InputReportKind::Unknown;
    next.motion_len = 0;
    next.motion_offset = 0;
    std::memset(next.motion, 0, sizeof(next.motion));
    clear_battery(&next);

    bool ok = false;
    if (kind == InputReportKind::Fd2) {
        ok = parse_fd2(data, len, &next);
    } else if (kind == InputReportKind::Legacy) {
        ok = parse_legacy(data, len, &next);
    } else if (len >= 16) {
        next.kind = InputReportKind::Fd2;
        ok = parse_fd2(data, len, &next);
    } else {
        next.kind = InputReportKind::Legacy;
        ok = parse_legacy(data, len, &next);
    }

    if (!ok) {
        s_input.parse_errors++;
        return false;
    }

    next.valid = true;
    next.updates = s_input.updates + 1;
    next.parse_errors = s_input.parse_errors;
    s_input = next;
    s_last_input_us = esp_timer_get_time();
    return true;
}

bool input_get_snapshot(InputSnapshot *out) {
    if (out != nullptr) {
        *out = s_input;
    }
    return s_input.valid;
}

uint32_t input_updates() {
    return s_input.updates;
}

uint32_t input_parse_errors() {
    return s_input.parse_errors;
}

uint32_t input_last_age_ms() {
    if (s_last_input_us == 0) {
        return 0xffffffffu;
    }
    return static_cast<uint32_t>((esp_timer_get_time() - s_last_input_us) / 1000ULL);
}

const char *input_kind_name(InputReportKind kind) {
    switch (kind) {
    case InputReportKind::Fd2:
        return "FD2";
    case InputReportKind::Legacy:
        return "LEG";
    case InputReportKind::Unknown:
    default:
        return "UNK";
    }
}

void input_format_motion_json(const InputSnapshot *snapshot, char *out, size_t out_len) {
    if (out == nullptr || out_len == 0) {
        return;
    }
    if (snapshot == nullptr || !snapshot->valid || !snapshot->motion_valid) {
        std::snprintf(out,
                      out_len,
                      "\"motion_valid\":false,\"motion_updates\":%lu,"
                      "\"motion_kind\":\"UNK\",\"motion_len\":0,\"motion_offset\":0,"
                      "\"accel\":[0,0,0],\"gyro\":[0,0,0],\"motion_raw\":\"\"",
                      snapshot ? static_cast<unsigned long>(snapshot->motion_updates) : 0ul);
        return;
    }

    char raw_hex[kMotionSampleSize * 2 + 1] = {};
    for (size_t i = 0; i < kMotionSampleSize; ++i) {
        std::snprintf(raw_hex + i * 2, sizeof(raw_hex) - i * 2, "%02x", snapshot->motion[i]);
    }
    auto read16 = [](const uint8_t *p) {
        return static_cast<int16_t>(static_cast<uint16_t>(p[0]) |
                                    (static_cast<uint16_t>(p[1]) << 8));
    };

    std::snprintf(out,
                  out_len,
                  "\"motion_valid\":true,\"motion_updates\":%lu,"
                  "\"motion_kind\":\"%s\",\"motion_len\":%u,\"motion_offset\":%u,"
                  "\"accel\":[%d,%d,%d],\"gyro\":[%d,%d,%d],\"motion_raw\":\"%s\"",
                  static_cast<unsigned long>(snapshot->motion_updates),
                  input_kind_name(snapshot->motion_kind),
                  static_cast<unsigned>(snapshot->motion_len),
                  static_cast<unsigned>(snapshot->motion_offset),
                  static_cast<int>(read16(snapshot->motion + 0)),
                  static_cast<int>(read16(snapshot->motion + 2)),
                  static_cast<int>(read16(snapshot->motion + 4)),
                  static_cast<int>(read16(snapshot->motion + 6)),
                  static_cast<int>(read16(snapshot->motion + 8)),
                  static_cast<int>(read16(snapshot->motion + 10)),
                  raw_hex);
}

} // namespace ns2
