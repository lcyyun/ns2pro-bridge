#include "ns2_input.h"

#include <cstdio>
#include <cstring>

namespace {

constexpr uint16_t CENTER_12BIT = 2048;
constexpr int32_t AXIS_DEADZONE = 48;
constexpr uint32_t AXIS_CALIBRATION_SAMPLES = 20;
constexpr uint16_t FD2_FULL_REPORT_MIN_LEN = 60;
constexpr uint16_t FD2_FULL_MOTION_OFFSET = 48;

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
    ButtonCount
};

struct AxisCalibration {
    bool calibrated;
    uint32_t sample_count;
    uint32_t sum_lx;
    uint32_t sum_ly;
    uint32_t sum_rx;
    uint32_t sum_ry;
    uint16_t center_lx;
    uint16_t center_ly;
    uint16_t center_rx;
    uint16_t center_ry;
};

Ns2InputSnapshot input{};
AxisCalibration fd2_axis{};
AxisCalibration legacy_axis{};

void reset_axis(AxisCalibration *axis) {
    memset(axis, 0, sizeof(*axis));
    axis->center_lx = CENTER_12BIT;
    axis->center_ly = CENTER_12BIT;
    axis->center_rx = CENTER_12BIT;
    axis->center_ry = CENTER_12BIT;
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

void decode_motion_sample(const uint8_t raw[NS2_MOTION_SAMPLE_SIZE], Ns2MotionSample *out) {
    memset(out, 0, sizeof(*out));
    out->valid = true;
    memcpy(out->raw, raw, NS2_MOTION_SAMPLE_SIZE);
    for (uint8_t i = 0; i < 3; i++) {
        out->accel[i] = read_le16s(raw + i * 2);
        out->gyro[i] = read_le16s(raw + 6 + i * 2);
    }
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
    const uint16_t clamped = clamp12(static_cast<int32_t>(CENTER_12BIT) +
                                     static_cast<int32_t>(value) -
                                     static_cast<int32_t>(center));
    int32_t delta = static_cast<int32_t>(clamped) - CENTER_12BIT;
    if (delta < 0) {
        delta = -delta;
    }
    return delta <= AXIS_DEADZONE ? CENTER_12BIT : clamped;
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

void apply_axes(AxisCalibration *axis,
                Ns2InputSnapshot *state,
                uint16_t lx,
                uint16_t ly,
                uint16_t rx,
                uint16_t ry) {
    if (!axis->calibrated && state->buttons == 0) {
        axis->sum_lx += lx;
        axis->sum_ly += ly;
        axis->sum_rx += rx;
        axis->sum_ry += ry;
        axis->sample_count++;
        if (axis->sample_count >= AXIS_CALIBRATION_SAMPLES) {
            axis->center_lx = static_cast<uint16_t>(axis->sum_lx / axis->sample_count);
            axis->center_ly = static_cast<uint16_t>(axis->sum_ly / axis->sample_count);
            axis->center_rx = static_cast<uint16_t>(axis->sum_rx / axis->sample_count);
            axis->center_ry = static_cast<uint16_t>(axis->sum_ry / axis->sample_count);
            axis->calibrated = true;
            printf("[NS2 INPUT] auto center %s lx=%u ly=%u rx=%u ry=%u\n",
                   ns2_input_kind_name(state->kind),
                   axis->center_lx,
                   axis->center_ly,
                   axis->center_rx,
                   axis->center_ry);
        }
    }

    if (!axis->calibrated) {
        state->lx = CENTER_12BIT;
        state->ly = CENTER_12BIT;
        state->rx = CENTER_12BIT;
        state->ry = CENTER_12BIT;
        return;
    }

    state->lx = recenter_axis(lx, axis->center_lx);
    state->ly = recenter_axis(ly, axis->center_ly);
    state->rx = recenter_axis(rx, axis->center_rx);
    state->ry = recenter_axis(ry, axis->center_ry);
}

bool parse_fd2(const uint8_t *data, uint16_t len, Ns2InputSnapshot *out) {
    if (len < 16) {
        return false;
    }
    out->buttons = decode_fd2_buttons(read_le32(data + 4));
    apply_axes(&fd2_axis,
               out,
               unpack12_x(data, 10),
               unpack12_y(data, 10),
               unpack12_x(data, 13),
               unpack12_y(data, 13));
    if (len >= FD2_FULL_REPORT_MIN_LEN) {
        memcpy(out->motion, data + FD2_FULL_MOTION_OFFSET, NS2_MOTION_SAMPLE_SIZE);
        out->motion_valid = true;
        out->motion_updates++;
        out->motion_kind = Ns2InputReportKind::Fd2;
        out->motion_len = len;
        out->motion_offset = FD2_FULL_MOTION_OFFSET;
    }
    return true;
}

bool parse_legacy(const uint8_t *data, uint16_t len, Ns2InputSnapshot *out) {
    if (len < 11) {
        return false;
    }
    out->buttons = decode_legacy_buttons(data[2], data[3], data[4]);
    apply_axes(&legacy_axis,
               out,
               unpack12_x(data, 5),
               unpack12_y(data, 5),
               unpack12_x(data, 8),
               unpack12_y(data, 8));
    return true;
}

} // namespace

void ns2_input_reset() {
    memset(&input, 0, sizeof(input));
    input.lx = CENTER_12BIT;
    input.ly = CENTER_12BIT;
    input.rx = CENTER_12BIT;
    input.ry = CENTER_12BIT;
    reset_axis(&fd2_axis);
    reset_axis(&legacy_axis);
}

bool ns2_input_parse_notify(Ns2InputReportKind kind, const uint8_t *data, uint16_t len) {
    if (!data || len == 0) {
        input.parse_errors++;
        return false;
    }

    Ns2InputSnapshot next = input;
    next.valid = false;
    next.kind = kind;
    next.len = len;
    next.first_byte = data[0];
    next.raw_valid = true;
    next.raw_len = len > NS2_INPUT_RAW_REPORT_MAX ? NS2_INPUT_RAW_REPORT_MAX : len;
    memcpy(next.raw, data, next.raw_len);
    if (next.raw_len < sizeof(next.raw)) {
        memset(next.raw + next.raw_len, 0, sizeof(next.raw) - next.raw_len);
    }
    next.motion_valid = false;
    next.motion_kind = Ns2InputReportKind::Unknown;
    next.motion_len = 0;
    next.motion_offset = 0;
    memset(next.motion, 0, sizeof(next.motion));

    bool ok = false;
    if (kind == Ns2InputReportKind::Fd2) {
        ok = parse_fd2(data, len, &next);
    } else if (kind == Ns2InputReportKind::Legacy) {
        ok = parse_legacy(data, len, &next);
    } else if (len >= 16) {
        next.kind = Ns2InputReportKind::Fd2;
        ok = parse_fd2(data, len, &next);
    } else {
        next.kind = Ns2InputReportKind::Legacy;
        ok = parse_legacy(data, len, &next);
    }

    if (!ok) {
        input.parse_errors++;
        return false;
    }

    next.valid = true;
    next.updates = input.updates + 1;
    next.parse_errors = input.parse_errors;
    input = next;
    return true;
}

bool ns2_input_get_snapshot(Ns2InputSnapshot *out) {
    if (out) {
        *out = input;
    }
    return input.valid;
}

bool ns2_input_get_motion_sample(Ns2MotionSample *out) {
    if (!input.valid || !input.motion_valid) {
        if (out) {
            memset(out, 0, sizeof(*out));
        }
        return false;
    }
    if (out) {
        decode_motion_sample(input.motion, out);
    }
    return true;
}

const char *ns2_input_kind_name(Ns2InputReportKind kind) {
    switch (kind) {
        case Ns2InputReportKind::Fd2:
            return "FD2";
        case Ns2InputReportKind::Legacy:
            return "LEG";
        case Ns2InputReportKind::Unknown:
        default:
            return "UNK";
    }
}

void ns2_input_format_buttons(uint32_t buttons, char *out, size_t out_len) {
    static const char *names[] = {
        "B", "A", "Y", "X", "R", "ZR", "+", "RS",
        "DD", "DR", "DL", "DU", "L", "ZL", "-", "LS",
        "H", "CAP", "GR", "GL", "C"
    };
    if (!out || out_len == 0) {
        return;
    }
    size_t used = 0;
    out[0] = 0;
    for (uint8_t i = 0; i < ButtonCount; i++) {
        if ((buttons & (1u << i)) == 0) {
            continue;
        }
        const int written = snprintf(out + used,
                                     out_len > used ? out_len - used : 0,
                                     "%s%s",
                                     used == 0 ? "" : " ",
                                     names[i]);
        if (written < 0 || static_cast<size_t>(written) >= out_len - used) {
            out[out_len - 1] = 0;
            return;
        }
        used += static_cast<size_t>(written);
    }
    if (used == 0) {
        snprintf(out, out_len, "-");
    }
}

void ns2_input_format_motion_json(const Ns2InputSnapshot *snapshot, char *out, size_t out_len) {
    if (!out || out_len == 0) {
        return;
    }
    out[0] = 0;

    if (!snapshot || !snapshot->valid || !snapshot->motion_valid) {
        snprintf(out,
                 out_len,
                 "\"motion_valid\":false,\"motion_updates\":%lu,"
                 "\"motion_kind\":\"UNK\",\"motion_len\":0,\"motion_offset\":0,"
                 "\"accel\":[0,0,0],\"gyro\":[0,0,0],\"motion_raw\":\"\"",
                 snapshot ? static_cast<unsigned long>(snapshot->motion_updates) : 0ul);
        return;
    }

    Ns2MotionSample motion;
    decode_motion_sample(snapshot->motion, &motion);
    char raw_hex[NS2_MOTION_SAMPLE_SIZE * 2 + 1];
    for (size_t i = 0; i < NS2_MOTION_SAMPLE_SIZE; i++) {
        snprintf(raw_hex + i * 2, sizeof(raw_hex) - i * 2, "%02x", motion.raw[i]);
    }

    snprintf(out,
             out_len,
             "\"motion_valid\":true,"
             "\"motion_updates\":%lu,"
             "\"motion_kind\":\"%s\","
             "\"motion_len\":%u,"
             "\"motion_offset\":%u,"
             "\"accel\":[%d,%d,%d],"
             "\"gyro\":[%d,%d,%d],"
             "\"motion_raw\":\"%s\"",
             static_cast<unsigned long>(snapshot->motion_updates),
             ns2_input_kind_name(snapshot->motion_kind),
             static_cast<unsigned>(snapshot->motion_len),
             static_cast<unsigned>(snapshot->motion_offset),
             static_cast<int>(motion.accel[0]),
             static_cast<int>(motion.accel[1]),
             static_cast<int>(motion.accel[2]),
             static_cast<int>(motion.gyro[0]),
             static_cast<int>(motion.gyro[1]),
             static_cast<int>(motion.gyro[2]),
             raw_hex);
}
