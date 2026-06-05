#include "ns2_usb.h"

// Nintendo-style USB report mapping and HID OUT rumble forwarding are informed
// by y700-switch2-pro-bridge protocol experiments and adapted for TinyUSB.

#include <cstdlib>
#include <cstdio>
#include <cstring>

#include "ns2_ble.h"
#include "ns2_display.h"
#include "ns2_input.h"
#include "ns2_state.h"
#include "ns2_status.h"
#include "pico/bootrom.h"
#include "pico/time.h"
#include "tusb.h"

namespace {

constexpr uint16_t REPORT_RATE_DEFAULT_HZ = 250;
constexpr uint16_t REPORT_RATE_MIN_HZ = 60;
constexpr uint16_t REPORT_RATE_MAX_HZ = 1000;
constexpr uint8_t HID_INSTANCE = 0;
constexpr uint8_t VENDOR_INTERFACE = 0;
constexpr uint8_t REPORT_MOTION_OFFSET = 0x31;
constexpr uint8_t REPORT_TIMESTAMP_OFFSET = 0x2b;
constexpr size_t VENDOR_REPLY_MAX = 128;
constexpr size_t VENDOR_PENDING_MAX = 256;
constexpr uint16_t STICK_CENTER_12BIT = 2048;
constexpr uint16_t STICK_USB_CAL_RANGE_12BIT = 2048;
constexpr uint16_t STICK_INPUT_FULL_RANGE_12BIT = 1600;
constexpr uint16_t HD_SCALE_DEFAULT_PERCENT = 60;
constexpr uint32_t HD_HOLD_DEFAULT_US = 140000;
constexpr uint32_t HD_TICK_DEFAULT_US = 30000;
constexpr uint8_t HD_STOP_DEFAULT_PACKETS = 3;
constexpr uint16_t HD_SCALE_MIN_PERCENT = 5;
constexpr uint16_t HD_SCALE_MAX_PERCENT = 250;
constexpr uint16_t HD_HOLD_MIN_MS = 20;
constexpr uint16_t HD_HOLD_MAX_MS = 3000;
constexpr uint16_t HD_TICK_MIN_MS = 5;
constexpr uint16_t HD_TICK_MAX_MS = 100;
constexpr uint8_t HD_STOP_MIN_PACKETS = 1;
constexpr uint8_t HD_STOP_MAX_PACKETS = 12;
constexpr size_t FEATURE_REPLY_MAX = 1024;
constexpr char FEATURE_SET_MAGIC[] = "Y7HID1";
constexpr char FEATURE_REPLY_MAGIC[] = "Y7HRS1";

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
};

struct UsbRuntime {
    bool mounted;
    bool suspended;
    uint64_t next_report_us;
    uint16_t report_rate_hz;
    uint32_t report_interval_us;
    uint8_t report_seq;
    uint32_t reports_sent;
    uint32_t reports_failed;
    uint32_t raw_passthrough_reports;
    uint32_t parsed_reports;
    uint32_t hid_out_count;
    uint8_t hid_last_report_id;
    uint8_t hid_last_effective_report_id;
    uint8_t hid_last_type;
    uint16_t hid_last_len;
    uint8_t hid_last_first_byte;

    uint32_t vendor_out_count;
    uint32_t vendor_in_count;
    uint32_t vendor_in_done_count;
    uint16_t vendor_last_rx_len;
    uint16_t vendor_last_tx_len;
    uint8_t vendor_last_cmd;
    uint8_t vendor_last_arg;
    uint8_t pending_reply[VENDOR_PENDING_MAX];
    size_t pending_len;
    size_t pending_offset;
    uint8_t pending_itf;

    bool hd_stream_active;
    uint64_t hd_stream_until_us;
    uint64_t hd_next_tick_us;
    uint8_t hd_left_vibration[5];
    uint8_t hd_right_vibration[5];
    uint8_t hd_packet_id;
    uint8_t hd_stop_packets_pending;
    bool hd_enabled;
    uint16_t hd_scale_percent;
    uint32_t hd_hold_us;
    uint32_t hd_tick_us;
    uint8_t hd_stop_packets;
    uint32_t hd_stream_updates;
    uint32_t hd_stream_writes;
    uint32_t hd_stream_stops;
    uint32_t hd_stream_errors;

    uint8_t feature_reply[FEATURE_REPLY_MAX];
    uint16_t feature_reply_len;
    uint16_t feature_reply_offset;
    bool feature_reply_complete;
    uint32_t feature_set_count;
    uint32_t feature_get_count;
    bool bootrom_requested;
    char feature_last_command[64];
};

UsbRuntime usb{};

bool button_pressed(const Ns2InputSnapshot &input, Button button) {
    return (input.buttons & (1u << static_cast<uint8_t>(button))) != 0;
}

void write_bytes(uint8_t *out, size_t out_len, size_t offset, const uint8_t *data, size_t data_len) {
    if (!out || !data || offset >= out_len) {
        return;
    }
    size_t n = data_len;
    if (n > out_len - offset) {
        n = out_len - offset;
    }
    memcpy(out + offset, data, n);
}

void pack12_pair(uint8_t *out, size_t offset, uint16_t x, uint16_t y) {
    out[offset] = static_cast<uint8_t>(x & 0xff);
    out[offset + 1] = static_cast<uint8_t>(((x >> 8) & 0x0f) | ((y & 0x0f) << 4));
    out[offset + 2] = static_cast<uint8_t>((y >> 4) & 0xff);
}

void write_default_stick_calibration(uint8_t *data, size_t data_len) {
    uint8_t calib[9];
    pack12_pair(calib, 0, STICK_USB_CAL_RANGE_12BIT, STICK_USB_CAL_RANGE_12BIT);
    pack12_pair(calib, 3, STICK_CENTER_12BIT, STICK_CENTER_12BIT);
    pack12_pair(calib, 6, STICK_USB_CAL_RANGE_12BIT, STICK_USB_CAL_RANGE_12BIT);
    write_bytes(data, data_len, 0x28, calib, sizeof(calib));
}

uint16_t normalize_stick_axis(uint16_t value) {
    int32_t delta = static_cast<int32_t>(value) - STICK_CENTER_12BIT;
    if (delta == 0) {
        return STICK_CENTER_12BIT;
    }

    const int32_t rounding = delta > 0 ?
        STICK_INPUT_FULL_RANGE_12BIT / 2 :
        -(STICK_INPUT_FULL_RANGE_12BIT / 2);
    const int32_t scaled = (delta * STICK_CENTER_12BIT + rounding) /
                           STICK_INPUT_FULL_RANGE_12BIT;
    int32_t mapped = static_cast<int32_t>(STICK_CENTER_12BIT) + scaled;
    if (mapped < 0) {
        mapped = 0;
    } else if (mapped > 4095) {
        mapped = 4095;
    }
    return static_cast<uint16_t>(mapped);
}

int clamp_int(int value, int min_value, int max_value) {
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

uint32_t report_interval_from_rate(uint16_t rate_hz) {
    if (rate_hz == 0) {
        return 1000000u / REPORT_RATE_DEFAULT_HZ;
    }
    return (1000000u + rate_hz / 2u) / rate_hz;
}

void set_usb_report_rate(uint32_t rate_hz) {
    usb.report_rate_hz = static_cast<uint16_t>(clamp_int(static_cast<int>(rate_hz),
                                                         REPORT_RATE_MIN_HZ,
                                                         REPORT_RATE_MAX_HZ));
    usb.report_interval_us = report_interval_from_rate(usb.report_rate_hz);
    usb.next_report_us = time_us_64() + usb.report_interval_us;
    ns2_config_set_report_rate_hz(usb.report_rate_hz);
}

int map_switch_amp_to_ble(int value) {
    const int64_t scaled = static_cast<int64_t>(value) * 1023LL * usb.hd_scale_percent;
    const int64_t mapped = (scaled + 1450000LL) / 2900000LL;
    return clamp_int(static_cast<int>(mapped), 0, 1023);
}

void build_ble_vibration_data(uint16_t lf_freq,
                              bool lf_tone,
                              uint16_t lf_amp,
                              uint16_t hf_freq,
                              bool hf_tone,
                              uint16_t hf_amp,
                              uint8_t out[5]) {
    uint64_t value = 0;
    value |= static_cast<uint64_t>(lf_freq & 0x01ff);
    value |= static_cast<uint64_t>(lf_tone ? 1 : 0) << 9;
    value |= static_cast<uint64_t>(lf_amp & 0x03ff) << 10;
    value |= static_cast<uint64_t>(hf_freq & 0x01ff) << 20;
    value |= static_cast<uint64_t>(hf_tone ? 1 : 0) << 29;
    value |= static_cast<uint64_t>(hf_amp & 0x03ff) << 30;

    for (size_t i = 0; i < 5; i++) {
        out[i] = static_cast<uint8_t>((value >> (8 * i)) & 0xff);
    }
}

void build_zero_ble_vibration(uint8_t out[5]) {
    build_ble_vibration_data(0x0e1, false, 0, 0x1e1, false, 0, out);
}

void encode_ble_vibration_from_switch_frame(const uint8_t *report,
                                            uint16_t len,
                                            uint16_t offset,
                                            uint8_t out[5]) {
    if (len < offset + 5) {
        build_zero_ble_vibration(out);
        return;
    }

    const int b0 = report[offset];
    const int b1 = report[offset + 1];
    const int b2 = report[offset + 2];
    const int b3 = report[offset + 3];
    const int b4 = report[offset + 4];

    const int high_freq = b0 | ((b1 & 0x03) << 8);
    const int high_amp = ((b1 & 0xfc) << 4) | ((b2 & 0x0f) << 12);
    const int low_freq = ((b2 & 0xf0) >> 4) | ((b3 & 0x3f) << 4);
    const int low_amp = (b3 & 0xc0) | (b4 << 8);

    build_ble_vibration_data(static_cast<uint16_t>(low_freq),
                             false,
                             static_cast<uint16_t>(map_switch_amp_to_ble(low_amp)),
                             static_cast<uint16_t>(high_freq),
                             false,
                             static_cast<uint16_t>(map_switch_amp_to_ble(high_amp)),
                             out);
}

void write_motor_block(uint8_t *out, uint16_t offset, uint8_t packet_id, const uint8_t first[5], const uint8_t zero[5]) {
    out[offset] = static_cast<uint8_t>(0x50 | (packet_id & 0x0f));
    memcpy(out + offset + 1, first, 5);
    memcpy(out + offset + 6, zero, 5);
    memcpy(out + offset + 11, zero, 5);
}

void build_pro2_hd_packet(uint8_t packet_id, const uint8_t left[5], const uint8_t right[5], uint8_t out[33]) {
    uint8_t zero[5];
    build_zero_ble_vibration(zero);

    memset(out, 0, 33);
    out[0] = 0x00;
    write_motor_block(out, 1, packet_id, left, zero);
    write_motor_block(out, 17, packet_id, right, zero);
}

bool has_non_zero_payload(const uint8_t *data, uint16_t len, uint16_t offset) {
    for (uint16_t i = offset; i < len; i++) {
        if (data[i] != 0) {
            return true;
        }
    }
    return false;
}

bool has_neutral_rumble_frame(const uint8_t *data, uint16_t len, uint16_t offset) {
    return len >= offset + 5 &&
           data[offset] == 0x87 &&
           data[offset + 1] == 0x01 &&
           data[offset + 2] == 0x20 &&
           data[offset + 3] == 0x11 &&
           data[offset + 4] == 0x00;
}

bool is_neutral_switch_rumble(const uint8_t *data, uint16_t len) {
    return has_neutral_rumble_frame(data, len, 2) &&
           has_neutral_rumble_frame(data, len, 0x12);
}

bool is_switch2_hid_rumble_report(const uint8_t *data, uint16_t len) {
    return len >= 7 &&
           data[0] == NS2_USB_NINTENDO_OUTPUT_REPORT_ID &&
           (data[1] & 0xf0) == 0x50;
}

void stop_hd_rumble() {
    build_zero_ble_vibration(usb.hd_left_vibration);
    build_zero_ble_vibration(usb.hd_right_vibration);
    usb.hd_stream_until_us = 0;
    usb.hd_stream_active = false;
    usb.hd_stop_packets_pending = usb.hd_stop_packets;
    usb.hd_stream_stops++;
    usb.hd_next_tick_us = 0;
}

void update_hd_rumble_stream(const uint8_t left[5], const uint8_t right[5], uint32_t hold_us, const char *reason) {
    (void)reason;
    if (!usb.hd_enabled) {
        stop_hd_rumble();
        return;
    }
    const uint64_t now = time_us_64();
    memcpy(usb.hd_left_vibration, left, 5);
    memcpy(usb.hd_right_vibration, right, 5);
    usb.hd_stream_until_us = now + hold_us;
    usb.hd_stream_active = true;
    usb.hd_stream_updates++;
    usb.hd_next_tick_us = 0;
}

void bridge_hid_output_to_ble(const uint8_t *data, uint16_t len) {
    if (!data || len < 2) {
        return;
    }

    if (is_switch2_hid_rumble_report(data, len)) {
        const bool active = has_non_zero_payload(data, len, 2) &&
                            !is_neutral_switch_rumble(data, len);
        uint8_t left[5];
        uint8_t right[5];
        if (active) {
            encode_ble_vibration_from_switch_frame(data, len, 2, left);
            encode_ble_vibration_from_switch_frame(data, len, 0x12, right);
            update_hd_rumble_stream(left, right, usb.hd_hold_us, "hid-out");
        } else {
            stop_hd_rumble();
        }
    }
}

void hd_rumble_task() {
    const uint64_t now = time_us_64();
    if (usb.hd_next_tick_us != 0 && now < usb.hd_next_tick_us) {
        return;
    }

    uint8_t left[5];
    uint8_t right[5];
    bool send_stop = false;
    bool active = usb.hd_stream_active && now <= usb.hd_stream_until_us;

    if (usb.hd_stream_active && !active) {
        usb.hd_stream_active = false;
        usb.hd_stop_packets_pending = usb.hd_stop_packets;
        usb.hd_stream_stops++;
    }

    if (active) {
        memcpy(left, usb.hd_left_vibration, sizeof(left));
        memcpy(right, usb.hd_right_vibration, sizeof(right));
    } else if (usb.hd_stop_packets_pending > 0) {
        usb.hd_stop_packets_pending--;
        send_stop = true;
        build_zero_ble_vibration(left);
        build_zero_ble_vibration(right);
    } else {
        return;
    }

    uint8_t packet[33];
    build_pro2_hd_packet(usb.hd_packet_id++ & 0x0f, left, right, packet);
    if (ns2_ble_send_rumble(packet, sizeof(packet))) {
        usb.hd_stream_writes++;
    } else {
        usb.hd_stream_errors++;
    }
    usb.hd_next_tick_us = now + usb.hd_tick_us;

    if (send_stop && usb.hd_stop_packets_pending == 0) {
        usb.hd_next_tick_us = now + usb.hd_tick_us;
    }
}

void make_neutral_report(uint8_t report[NS2_USB_NINTENDO_REPORT_SIZE]) {
    memset(report, 0, NS2_USB_NINTENDO_REPORT_SIZE);
    report[0] = NS2_USB_NINTENDO_INPUT_REPORT_ID;
    report[2] = 0x20;
    report[11] = 0x00;
    report[12] = 0x08;
    report[13] = 0x80;
    report[14] = 0x00;
    report[15] = 0x08;
    report[16] = 0x80;
}

void write_sensor_timestamp(uint8_t report[NS2_USB_NINTENDO_REPORT_SIZE]) {
    const uint32_t now = static_cast<uint32_t>(time_us_64());
    report[REPORT_TIMESTAMP_OFFSET] = static_cast<uint8_t>(now & 0xff);
    report[REPORT_TIMESTAMP_OFFSET + 1] = static_cast<uint8_t>((now >> 8) & 0xff);
    report[REPORT_TIMESTAMP_OFFSET + 2] = static_cast<uint8_t>((now >> 16) & 0xff);
    report[REPORT_TIMESTAMP_OFFSET + 3] = static_cast<uint8_t>((now >> 24) & 0xff);
}

void make_nintendo_report(const Ns2InputSnapshot *input, uint8_t report[NS2_USB_NINTENDO_REPORT_SIZE]) {
    make_neutral_report(report);

    if (!input || !input->valid) {
        report[1] = usb.report_seq++;
        write_sensor_timestamp(report);
        return;
    }

    if (ns2_config_usb_raw_passthrough_enabled() &&
        input->kind == Ns2InputReportKind::Fd2 &&
        input->raw_valid &&
        input->raw_len == NS2_USB_NINTENDO_REPORT_SIZE - 1) {
        memcpy(report + 1, input->raw, input->raw_len);
        usb.raw_passthrough_reports++;
        return;
    }

    if (input) {
        usb.parsed_reports++;
    }

    report[1] = usb.report_seq++;

    if (button_pressed(*input, ButtonY)) report[5] |= 0x01;
    if (button_pressed(*input, ButtonX)) report[5] |= 0x02;
    if (button_pressed(*input, ButtonB)) report[5] |= 0x04;
    if (button_pressed(*input, ButtonA)) report[5] |= 0x08;
    if (button_pressed(*input, ButtonR)) report[5] |= 0x40;
    if (button_pressed(*input, ButtonZR)) report[5] |= 0x80;

    if (button_pressed(*input, ButtonMinus)) report[6] |= 0x01;
    if (button_pressed(*input, ButtonPlus)) report[6] |= 0x02;
    if (button_pressed(*input, ButtonRStick)) report[6] |= 0x04;
    if (button_pressed(*input, ButtonLStick)) report[6] |= 0x08;
    if (button_pressed(*input, ButtonHome)) report[6] |= 0x10;
    if (button_pressed(*input, ButtonCapture)) report[6] |= 0x20;
    if (button_pressed(*input, ButtonC)) report[6] |= 0x40;

    if (button_pressed(*input, ButtonDDown)) report[7] |= 0x01;
    if (button_pressed(*input, ButtonDUp)) report[7] |= 0x02;
    if (button_pressed(*input, ButtonDRight)) report[7] |= 0x04;
    if (button_pressed(*input, ButtonDLeft)) report[7] |= 0x08;
    if (button_pressed(*input, ButtonL)) report[7] |= 0x40;
    if (button_pressed(*input, ButtonZL)) report[7] |= 0x80;

    if (button_pressed(*input, ButtonGR)) report[8] |= 0x01;
    if (button_pressed(*input, ButtonGL)) report[8] |= 0x02;

    pack12_pair(report,
                11,
                normalize_stick_axis(input->lx),
                normalize_stick_axis(input->ly));
    pack12_pair(report,
                14,
                normalize_stick_axis(input->rx),
                normalize_stick_axis(input->ry));
    write_sensor_timestamp(report);

    if (input->motion_valid && REPORT_MOTION_OFFSET + NS2_MOTION_SAMPLE_SIZE <= NS2_USB_NINTENDO_REPORT_SIZE) {
        memcpy(report + REPORT_MOTION_OFFSET, input->motion, NS2_MOTION_SAMPLE_SIZE);
    }
}

bool live_input_snapshot(Ns2InputSnapshot *out) {
    if (ns2_status_get_state() != Ns2BleState::ConnectedLive || !ns2_status_live_notify()) {
        if (out) {
            memset(out, 0, sizeof(*out));
        }
        return false;
    }
    return ns2_input_get_snapshot(out);
}

bool send_report_now() {
    if (!usb.mounted || usb.suspended || !tud_hid_n_ready(HID_INSTANCE)) {
        return false;
    }

    Ns2InputSnapshot input{};
    const bool live = live_input_snapshot(&input);
    uint8_t report[NS2_USB_NINTENDO_REPORT_SIZE];
    make_nintendo_report(live ? &input : nullptr, report);

    const bool ok = tud_hid_n_report(HID_INSTANCE,
                                     NS2_USB_NINTENDO_INPUT_REPORT_ID,
                                     report + 1,
                                     NS2_USB_NINTENDO_REPORT_SIZE - 1);
    if (ok) {
        usb.reports_sent++;
    } else {
        usb.reports_failed++;
    }
    return ok;
}

uint32_t command_address(const uint8_t *cmd, uint16_t cmd_len) {
    if (!cmd || cmd_len < 16) {
        return 0;
    }
    return static_cast<uint32_t>(cmd[12]) |
           (static_cast<uint32_t>(cmd[13]) << 8) |
           (static_cast<uint32_t>(cmd[14]) << 16) |
           (static_cast<uint32_t>(cmd[15]) << 24);
}

size_t flash_read_length(uint32_t address) {
    if (address == 0x13040) {
        return 0x10;
    }
    if (address == 0x13100) {
        return 0x18;
    }
    if (address == 0x13060) {
        return 0x20;
    }
    return 0x40;
}

size_t build_ack(const uint8_t *cmd, uint16_t cmd_len, uint8_t *reply, size_t reply_len) {
    if (!cmd || !reply || reply_len == 0) {
        return 0;
    }
    memset(reply, 0, reply_len);
    reply[0] = cmd[0];
    if (reply_len > 1) {
        reply[1] = 0x01;
    }
    if (reply_len > 2 && cmd_len > 2) {
        reply[2] = cmd[2];
    }
    if (reply_len > 3 && cmd_len > 3) {
        reply[3] = cmd[3];
    }
    if (reply_len > 4 && cmd_len > 4) {
        reply[4] = cmd[4];
    }
    if (reply_len > 5) {
        reply[5] = 0xf8;
    }
    return reply_len;
}

size_t build_flash_read_reply(const uint8_t *cmd, uint16_t cmd_len, uint8_t *reply, size_t reply_max) {
    if (cmd_len < 16 || !reply) {
        return 0;
    }

    const uint32_t address = command_address(cmd, cmd_len);
    const size_t data_len = flash_read_length(address);
    size_t full_reply_len = 0x10 + data_len;
    if (full_reply_len > reply_max) {
        return 0;
    }

    memset(reply, 0, full_reply_len);
    uint8_t *data = reply + 0x10;

    if (address == 0x13000) {
        static const uint8_t serial[] = {'H', 'A', '2', 'F', '8', '3', 'J', 'F'};
        write_bytes(data, data_len, 2, serial, sizeof(serial));
    }

    if (address == 0x13080 || address == 0x130C0) {
        memset(data, 0xff, data_len);
        write_default_stick_calibration(data, data_len);
    }

    if (address == 0x1fc040 || address == 0x1fc080 || address == 0x13060) {
        memset(data, 0xff, data_len);
    }

    if (address == 0x13040) {
        static const uint8_t block[] = {
            0x16, 0xf4, 0xd3, 0x41, 0x48, 0xce, 0x85, 0xba,
            0xf1, 0x05, 0x71, 0xba, 0x1f, 0x27, 0xcb, 0x3b,
        };
        write_bytes(data, data_len, 0, block, sizeof(block));
    }

    if (address == 0x13100) {
        static const uint8_t block[] = {
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x2d, 0x10, 0xa7, 0x3d,
            0xe7, 0x49, 0x35, 0x3c, 0xa4, 0x2d, 0x20, 0x41,
        };
        write_bytes(data, data_len, 0, block, sizeof(block));
    }

    reply[0] = 0x02;
    reply[1] = 0x01;
    reply[2] = cmd[2];
    reply[3] = cmd[3];
    reply[5] = 0xf8;
    reply[8] = static_cast<uint8_t>(data_len);
    memcpy(reply + 12, cmd + 12, 4);
    if (full_reply_len > 0x50) {
        full_reply_len = 0x50;
    }
    return full_reply_len;
}

size_t build_vendor_reply(const uint8_t *cmd, uint16_t cmd_len, uint8_t *reply, size_t reply_max) {
    if (!cmd || cmd_len == 0 || !reply || reply_max == 0) {
        return 0;
    }

    const uint8_t c0 = cmd[0];
    const uint8_t arg1_hi = cmd_len > 3 ? cmd[3] : 0;

    if (cmd_len >= 16 && c0 == 0x02) {
        return build_flash_read_reply(cmd, cmd_len, reply, reply_max);
    }
    if (c0 == 0x0c && arg1_hi == 0x02) {
        return 0;
    }
    if (c0 == 0x10) {
        return 0;
    }
    if (c0 == 0x03 && arg1_hi == 0x0d) {
        size_t n = build_ack(cmd, cmd_len, reply, 12);
        reply[8] = 0x01;
        return n;
    }
    if (c0 == 0x15 && arg1_hi == 0x01) {
        static const uint8_t mac_le[] = {0x2d, 0xfc, 0x27, 0xce, 0xc6, 0x38};
        size_t n = build_ack(cmd, cmd_len, reply, 17);
        reply[8] = 0x01;
        reply[9] = 0x04;
        reply[10] = 0x01;
        write_bytes(reply, n, 11, mac_le, sizeof(mac_le));
        return n;
    }
    if (c0 == 0x15 && arg1_hi == 0x02) {
        size_t n = build_ack(cmd, cmd_len, reply, 25);
        reply[8] = 0x01;
        return n;
    }
    if (c0 == 0x15 && arg1_hi == 0x03) {
        size_t n = build_ack(cmd, cmd_len, reply, 9);
        reply[8] = 0x01;
        return n;
    }
    if (c0 == 0x11) {
        static const uint8_t payload[] = {
            0x20, 0x03, 0x00, 0x00, 0x0a, 0xe8, 0x1c, 0x3b,
            0x79, 0x7d, 0x8b, 0x3a, 0x0a, 0xe8, 0x9c, 0x42,
            0x58, 0xa0, 0x0b, 0x42, 0x0a, 0xe8, 0x9c, 0x41,
            0x58, 0xa0, 0x0b, 0x41,
        };
        size_t n = build_ack(cmd, cmd_len, reply, 37);
        reply[8] = 0x01;
        write_bytes(reply, n, 9, payload, sizeof(payload));
        return n;
    }
    if (c0 == 0x01 && arg1_hi == 0x0c) {
        static const uint8_t payload[] = {0x61, 0x12, 0x50, 0x10};
        size_t n = build_ack(cmd, cmd_len, reply, 12);
        write_bytes(reply, n, 8, payload, sizeof(payload));
        return n;
    }
    if (c0 == 0x03 && arg1_hi == 0x01) {
        size_t n = build_ack(cmd, cmd_len, reply, 16);
        reply[10] = 0x40;
        reply[11] = 0xf0;
        reply[14] = 0x60;
        return n;
    }

    return build_ack(cmd, cmd_len, reply, 8);
}

void flush_vendor_reply() {
    if (usb.pending_len == 0 || usb.pending_offset >= usb.pending_len) {
        usb.pending_len = 0;
        usb.pending_offset = 0;
        return;
    }
    if (!tud_vendor_n_mounted(usb.pending_itf)) {
        return;
    }

    while (usb.pending_offset < usb.pending_len) {
        const uint32_t available = tud_vendor_n_write_available(usb.pending_itf);
        if (available == 0) {
            break;
        }
        const size_t remaining = usb.pending_len - usb.pending_offset;
        const uint32_t chunk = remaining > available ? available : static_cast<uint32_t>(remaining);
        const uint32_t written = tud_vendor_n_write(usb.pending_itf,
                                                    usb.pending_reply + usb.pending_offset,
                                                    chunk);
        if (written == 0) {
            break;
        }
        usb.pending_offset += written;
    }

    tud_vendor_n_write_flush(usb.pending_itf);
    if (usb.pending_offset >= usb.pending_len) {
        usb.pending_len = 0;
        usb.pending_offset = 0;
    }
}

void queue_vendor_reply(uint8_t itf, const uint8_t *reply, size_t reply_len) {
    if (!reply || reply_len == 0 || reply_len > sizeof(usb.pending_reply)) {
        return;
    }
    memcpy(usb.pending_reply, reply, reply_len);
    usb.pending_len = reply_len;
    usb.pending_offset = 0;
    usb.pending_itf = itf;
    usb.vendor_in_count++;
    usb.vendor_last_tx_len = static_cast<uint16_t>(reply_len);
    flush_vendor_reply();
}

const char *skip_spaces(const char *text) {
    while (text && (*text == ' ' || *text == '\t')) {
        text++;
    }
    return text ? text : "";
}

bool command_is(const char *line, const char *literal) {
    return strcmp(line, literal) == 0;
}

bool command_has_prefix(const char *line, const char *prefix) {
    const size_t len = strlen(prefix);
    return strncmp(line, prefix, len) == 0 &&
           (line[len] == 0 || line[len] == ' ' || line[len] == '\t');
}

bool parse_next_uint(const char **cursor, uint32_t *out) {
    if (!cursor || !out) {
        return false;
    }
    const char *p = skip_spaces(*cursor);
    if (*p < '0' || *p > '9') {
        return false;
    }

    char *end = nullptr;
    const unsigned long value = strtoul(p, &end, 10);
    if (end == p) {
        return false;
    }
    *out = static_cast<uint32_t>(value);
    *cursor = end;
    return true;
}

void queue_feature_json(const char *json) {
    if (!json) {
        json = "{\"ok\":false,\"error\":\"empty_reply\"}";
    }

    size_t len = strlen(json);
    if (len > sizeof(usb.feature_reply)) {
        static const char too_large[] = "{\"ok\":false,\"error\":\"reply_too_large\"}";
        json = too_large;
        len = strlen(json);
    }

    memcpy(usb.feature_reply, json, len);
    usb.feature_reply_len = static_cast<uint16_t>(len);
    usb.feature_reply_offset = 0;
    usb.feature_reply_complete = false;
}

void queue_feature_error(const char *error) {
    char json[128];
    snprintf(json, sizeof(json), "{\"ok\":false,\"profile\":\"ns2pro\",\"error\":\"%s\"}", error);
    queue_feature_json(json);
}

void format_rumble_config_json(char *out, size_t out_len) {
    snprintf(out,
             out_len,
             "{\"ok\":true,\"profile\":\"ns2pro\",\"rumble_enabled\":%s,"
             "\"rumble_active\":%s,\"scale_percent\":%u,\"hold_ms\":%u,"
             "\"tick_ms\":%u,\"stop_packets\":%u,\"updates\":%lu,"
             "\"writes\":%lu,\"stops\":%lu,\"errors\":%lu}",
             usb.hd_enabled ? "true" : "false",
             usb.hd_stream_active ? "true" : "false",
             static_cast<unsigned>(usb.hd_scale_percent),
             static_cast<unsigned>(usb.hd_hold_us / 1000),
             static_cast<unsigned>(usb.hd_tick_us / 1000),
             static_cast<unsigned>(usb.hd_stop_packets),
             static_cast<unsigned long>(usb.hd_stream_updates),
             static_cast<unsigned long>(usb.hd_stream_writes),
             static_cast<unsigned long>(usb.hd_stream_stops),
             static_cast<unsigned long>(usb.hd_stream_errors));
}

void format_settings_json(char *out, size_t out_len, bool saved = false) {
    snprintf(out,
             out_len,
             "{\"ok\":true,\"profile\":\"ns2pro\",\"settings\":true,"
             "\"saved\":%s,\"display_enabled\":%s,"
             "\"usb_raw_passthrough\":%s,\"web_parse_reports\":%s,"
             "\"rumble_enabled\":%s,\"report_rate_hz\":%u,"
             "\"rumble_scale_percent\":%u,\"rumble_hold_ms\":%u,"
             "\"rumble_tick_ms\":%u,\"rumble_stop_packets\":%u}",
             saved ? "true" : "false",
             ns2_config_display_enabled() ? "true" : "false",
             ns2_config_usb_raw_passthrough_enabled() ? "true" : "false",
             ns2_config_web_parse_reports_enabled() ? "true" : "false",
             ns2_config_rumble_enabled() ? "true" : "false",
             static_cast<unsigned>(ns2_config_report_rate_hz()),
             static_cast<unsigned>(ns2_config_rumble_scale_percent()),
             static_cast<unsigned>(ns2_config_rumble_hold_ms()),
             static_cast<unsigned>(ns2_config_rumble_tick_ms()),
             static_cast<unsigned>(ns2_config_rumble_stop_packets()));
}

void format_usb_status_json(char *out, size_t out_len) {
    Ns2UsbStats stats;
    ns2_usb_get_stats(&stats);
    snprintf(out,
             out_len,
             "{\"ok\":true,\"profile\":\"ns2pro\",\"usb_mounted\":%s,"
             "\"usb_suspended\":%s,\"reports_sent\":%lu,\"reports_failed\":%lu,"
             "\"raw_passthrough_reports\":%lu,\"parsed_reports\":%lu,"
             "\"report_rate_hz\":%u,\"report_interval_us\":%lu,"
             "\"hid_out\":%lu,\"hid_last_report\":\"0x%02x\","
             "\"hid_last_type\":%u,\"vendor_out\":%lu,\"vendor_in\":%lu,"
             "\"rumble_enabled\":%s,\"rumble_active\":%s,"
             "\"rumble_updates\":%lu,\"rumble_writes\":%lu,"
             "\"rumble_stops\":%lu,\"rumble_errors\":%lu,"
             "\"rumble_scale_percent\":%u,\"rumble_hold_ms\":%u,"
             "\"rumble_tick_ms\":%u,\"rumble_stop_packets\":%u,"
             "\"display_enabled\":%s,\"usb_raw_passthrough\":%s,"
             "\"web_parse_reports\":%s,"
             "\"feature_set\":%lu,\"feature_get\":%lu}",
             stats.mounted ? "true" : "false",
             stats.suspended ? "true" : "false",
             static_cast<unsigned long>(stats.reports_sent),
             static_cast<unsigned long>(stats.reports_failed),
             static_cast<unsigned long>(stats.raw_passthrough_reports),
             static_cast<unsigned long>(stats.parsed_reports),
             static_cast<unsigned>(stats.report_rate_hz),
             static_cast<unsigned long>(stats.report_interval_us),
             static_cast<unsigned long>(stats.hid_out_count),
             stats.hid_last_effective_report_id,
             static_cast<unsigned>(stats.hid_last_type),
             static_cast<unsigned long>(stats.vendor_out_count),
             static_cast<unsigned long>(stats.vendor_in_count),
             stats.rumble_enabled ? "true" : "false",
             stats.rumble_active ? "true" : "false",
             static_cast<unsigned long>(stats.rumble_updates),
             static_cast<unsigned long>(stats.rumble_writes),
             static_cast<unsigned long>(stats.rumble_stops),
             static_cast<unsigned long>(stats.rumble_errors),
             static_cast<unsigned>(stats.rumble_scale_percent),
             static_cast<unsigned>(stats.rumble_hold_ms),
             static_cast<unsigned>(stats.rumble_tick_ms),
             static_cast<unsigned>(stats.rumble_stop_packets),
             ns2_config_display_enabled() ? "true" : "false",
             ns2_config_usb_raw_passthrough_enabled() ? "true" : "false",
             ns2_config_web_parse_reports_enabled() ? "true" : "false",
             static_cast<unsigned long>(stats.feature_set_count),
             static_cast<unsigned long>(stats.feature_get_count));
}

void queue_motion_status_json() {
    Ns2InputSnapshot input;
    const bool valid = ns2_input_get_snapshot(&input);
    char motion_json[384];
    ns2_input_format_motion_json(valid ? &input : nullptr, motion_json, sizeof(motion_json));

    char json[768];
    const int written = snprintf(json,
                                 sizeof(json),
                                 "{\"ok\":true,\"profile\":\"ns2pro\",\"input_valid\":%s,"
                                 "\"kind\":\"%s\",\"len\":%u,\"updates\":%lu,"
                                 "\"buttons\":%lu,\"lx\":%u,\"ly\":%u,\"rx\":%u,\"ry\":%u,%s}",
                                 valid ? "true" : "false",
                                 valid ? ns2_input_kind_name(input.kind) : "UNK",
                                 valid ? static_cast<unsigned>(input.len) : 0u,
                                 valid ? static_cast<unsigned long>(input.updates) : 0ul,
                                 valid ? static_cast<unsigned long>(input.buttons) : 0ul,
                                 valid ? static_cast<unsigned>(input.lx) : 2048u,
                                 valid ? static_cast<unsigned>(input.ly) : 2048u,
                                 valid ? static_cast<unsigned>(input.rx) : 2048u,
                                 valid ? static_cast<unsigned>(input.ry) : 2048u,
                                 motion_json);
    if (written < 0 || static_cast<size_t>(written) >= sizeof(json)) {
        queue_feature_error("motion_status_too_large");
        return;
    }
    queue_feature_json(json);
}

void set_rumble_tune(uint32_t scale_percent,
                     uint32_t hold_ms,
                     uint32_t tick_ms,
                     uint32_t stop_packets) {
    usb.hd_scale_percent = static_cast<uint16_t>(clamp_int(static_cast<int>(scale_percent),
                                                           HD_SCALE_MIN_PERCENT,
                                                           HD_SCALE_MAX_PERCENT));
    const uint16_t hold = static_cast<uint16_t>(clamp_int(static_cast<int>(hold_ms),
                                                         HD_HOLD_MIN_MS,
                                                         HD_HOLD_MAX_MS));
    const uint16_t tick = static_cast<uint16_t>(clamp_int(static_cast<int>(tick_ms),
                                                         HD_TICK_MIN_MS,
                                                         HD_TICK_MAX_MS));
    usb.hd_hold_us = static_cast<uint32_t>(hold) * 1000u;
    usb.hd_tick_us = static_cast<uint32_t>(tick) * 1000u;
    usb.hd_stop_packets = static_cast<uint8_t>(clamp_int(static_cast<int>(stop_packets),
                                                         HD_STOP_MIN_PACKETS,
                                                         HD_STOP_MAX_PACKETS));
    ns2_config_set_rumble_tune(usb.hd_scale_percent,
                               hold,
                               tick,
                               usb.hd_stop_packets);
}

void format_usb_config_json(char *out, size_t out_len) {
    snprintf(out,
             out_len,
             "{\"ok\":true,\"profile\":\"ns2pro\",\"report_rate_hz\":%u,"
             "\"report_interval_us\":%lu,\"reports_sent\":%lu,"
             "\"reports_failed\":%lu}",
             static_cast<unsigned>(usb.report_rate_hz),
             static_cast<unsigned long>(usb.report_interval_us),
             static_cast<unsigned long>(usb.reports_sent),
             static_cast<unsigned long>(usb.reports_failed));
}

void handle_usb_command(const char *command) {
    char json[384];

    if (command_is(command, "usb config") || command_is(command, "report config")) {
        format_usb_config_json(json, sizeof(json));
        queue_feature_json(json);
        return;
    }
    if (command_is(command, "usb raw on")) {
        ns2_config_set_usb_raw_passthrough_enabled(true);
        format_settings_json(json, sizeof(json));
        queue_feature_json(json);
        return;
    }
    if (command_is(command, "usb raw off")) {
        ns2_config_set_usb_raw_passthrough_enabled(false);
        format_settings_json(json, sizeof(json));
        queue_feature_json(json);
        return;
    }
    if (command_has_prefix(command, "usb rate") || command_has_prefix(command, "report rate")) {
        const char *cursor = command_has_prefix(command, "usb rate") ?
            command + strlen("usb rate") :
            command + strlen("report rate");
        uint32_t rate = 0;
        if (!parse_next_uint(&cursor, &rate)) {
            queue_feature_error("usage: usb rate hz");
            return;
        }
        set_usb_report_rate(rate);
        format_usb_config_json(json, sizeof(json));
        queue_feature_json(json);
        return;
    }

    queue_feature_error("unknown_usb_command");
}

void handle_settings_command(const char *command) {
    char json[384];

    if (command_is(command, "settings") ||
        command_is(command, "settings status") ||
        command_is(command, "config") ||
        command_is(command, "config status")) {
        format_settings_json(json, sizeof(json));
        queue_feature_json(json);
        return;
    }
    if (command_is(command, "settings save") || command_is(command, "config save")) {
        const bool ok = ns2_config_save();
        if (ok) {
            format_settings_json(json, sizeof(json), true);
            queue_feature_json(json);
        } else {
            queue_feature_error("settings_save_failed");
        }
        return;
    }
    if (command_is(command, "display on") || command_is(command, "screen on")) {
        ns2_display_set_enabled(true);
        format_settings_json(json, sizeof(json));
        queue_feature_json(json);
        return;
    }
    if (command_is(command, "display off") || command_is(command, "screen off")) {
        ns2_display_set_enabled(false);
        format_settings_json(json, sizeof(json));
        queue_feature_json(json);
        return;
    }
    if (command_is(command, "web parse on") || command_is(command, "webui parse on")) {
        ns2_config_set_web_parse_reports_enabled(true);
        format_settings_json(json, sizeof(json));
        queue_feature_json(json);
        return;
    }
    if (command_is(command, "web parse off") || command_is(command, "webui parse off")) {
        ns2_config_set_web_parse_reports_enabled(false);
        format_settings_json(json, sizeof(json));
        queue_feature_json(json);
        return;
    }

    queue_feature_error("unknown_settings_command");
}

void start_test_rumble(bool left_on, bool right_on, uint16_t hold_ms, uint16_t amp) {
    uint8_t left[5];
    uint8_t right[5];
    build_zero_ble_vibration(left);
    build_zero_ble_vibration(right);

    const int scaled_amp = static_cast<int>(amp) * usb.hd_scale_percent / HD_SCALE_DEFAULT_PERCENT;
    const uint16_t clamped_amp = static_cast<uint16_t>(clamp_int(scaled_amp, 0, 1023));
    if (left_on) {
        build_ble_vibration_data(0x0e1, false, clamped_amp, 0x1e1, false, clamped_amp, left);
    }
    if (right_on) {
        build_ble_vibration_data(0x0e1, false, clamped_amp, 0x1e1, false, clamped_amp, right);
    }

    const uint32_t hold_us = static_cast<uint32_t>(clamp_int(hold_ms,
                                                            HD_HOLD_MIN_MS,
                                                            HD_HOLD_MAX_MS)) * 1000u;
    update_hd_rumble_stream(left, right, hold_us, "feature-test");
}

void handle_rumble_command(const char *command) {
    char json[256];

    if (command_is(command, "rumble config")) {
        format_rumble_config_json(json, sizeof(json));
        queue_feature_json(json);
        return;
    }
    if (command_is(command, "rumble stop")) {
        stop_hd_rumble();
        format_rumble_config_json(json, sizeof(json));
        queue_feature_json(json);
        return;
    }
    if (command_is(command, "rumble on")) {
        usb.hd_enabled = true;
        ns2_config_set_rumble_enabled(true);
        format_rumble_config_json(json, sizeof(json));
        queue_feature_json(json);
        return;
    }
    if (command_is(command, "rumble off")) {
        usb.hd_enabled = false;
        ns2_config_set_rumble_enabled(false);
        stop_hd_rumble();
        format_rumble_config_json(json, sizeof(json));
        queue_feature_json(json);
        return;
    }
    if (command_has_prefix(command, "rumble tune")) {
        const char *cursor = command + strlen("rumble tune");
        uint32_t scale = 0;
        uint32_t hold = 0;
        uint32_t tick = 0;
        uint32_t stops = 0;
        if (!parse_next_uint(&cursor, &scale) ||
            !parse_next_uint(&cursor, &hold) ||
            !parse_next_uint(&cursor, &tick) ||
            !parse_next_uint(&cursor, &stops)) {
            queue_feature_error("usage: rumble tune scale hold_ms tick_ms stop_packets");
            return;
        }
        set_rumble_tune(scale, hold, tick, stops);
        format_rumble_config_json(json, sizeof(json));
        queue_feature_json(json);
        return;
    }
    if (command_has_prefix(command, "rumble hold")) {
        const char *cursor = command + strlen("rumble hold");
        uint32_t hold = 0;
        if (!parse_next_uint(&cursor, &hold)) {
            hold = usb.hd_hold_us / 1000;
        }
        start_test_rumble(true, true, static_cast<uint16_t>(hold), 480);
        format_rumble_config_json(json, sizeof(json));
        queue_feature_json(json);
        return;
    }
    if (command_is(command, "rumble hdtest") || command_is(command, "rumble test both")) {
        start_test_rumble(true, true, 240, 520);
        format_rumble_config_json(json, sizeof(json));
        queue_feature_json(json);
        return;
    }
    if (command_is(command, "rumble test left")) {
        start_test_rumble(true, false, 240, 520);
        format_rumble_config_json(json, sizeof(json));
        queue_feature_json(json);
        return;
    }
    if (command_is(command, "rumble test right")) {
        start_test_rumble(false, true, 240, 520);
        format_rumble_config_json(json, sizeof(json));
        queue_feature_json(json);
        return;
    }
    if (command_is(command, "rumble test click")) {
        start_test_rumble(true, true, 70, 760);
        format_rumble_config_json(json, sizeof(json));
        queue_feature_json(json);
        return;
    }

    queue_feature_error("unknown_rumble_command");
}

void handle_feature_command(const char *raw_command) {
    const char *command = skip_spaces(raw_command);
    if (command[0] == 0) {
        queue_feature_error("empty_command");
        return;
    }

    if (command_is(command, "bootrom") || command_is(command, "bootsel")) {
        usb.bootrom_requested = true;
        queue_feature_json("{\"ok\":true,\"profile\":\"ns2pro\",\"bootrom\":true}");
        return;
    }
    if (command_is(command, "status") || command_is(command, "ns2 status")) {
        char json[512];
        ns2_status_format_json(json, sizeof(json));
        queue_feature_json(json);
        return;
    }
    if (command_is(command, "usb status")) {
        char json[768];
        format_usb_status_json(json, sizeof(json));
        queue_feature_json(json);
        return;
    }
    if (command_has_prefix(command, "settings") ||
        command_has_prefix(command, "config") ||
        command_has_prefix(command, "display") ||
        command_has_prefix(command, "screen") ||
        command_has_prefix(command, "web parse") ||
        command_has_prefix(command, "webui parse")) {
        handle_settings_command(command);
        return;
    }
    if (command_has_prefix(command, "usb") || command_has_prefix(command, "report")) {
        handle_usb_command(command);
        return;
    }
    if (command_is(command, "motion status") || command_is(command, "imu status")) {
        queue_motion_status_json();
        return;
    }
    if (command_has_prefix(command, "rumble")) {
        handle_rumble_command(command);
        return;
    }

    queue_feature_error("unknown_command");
}

void receive_feature_command(const uint8_t *payload, uint16_t payload_size) {
    usb.feature_set_count++;

    if (!payload || payload_size == 0) {
        queue_feature_error("empty_feature_report");
        return;
    }
    if (payload_size > 0 && payload[0] == NS2_USB_MANAGER_FEATURE_REPORT_ID) {
        payload++;
        payload_size--;
    }
    if (payload_size < strlen(FEATURE_SET_MAGIC) ||
        memcmp(payload, FEATURE_SET_MAGIC, strlen(FEATURE_SET_MAGIC)) != 0) {
        queue_feature_error("bad_magic");
        return;
    }

    payload += strlen(FEATURE_SET_MAGIC);
    payload_size = static_cast<uint16_t>(payload_size - strlen(FEATURE_SET_MAGIC));

    while (payload_size > 0 && payload[payload_size - 1] == 0) {
        payload_size--;
    }

    const size_t copy_len = payload_size < sizeof(usb.feature_last_command) - 1 ?
        payload_size :
        sizeof(usb.feature_last_command) - 1;
    memcpy(usb.feature_last_command, payload, copy_len);
    usb.feature_last_command[copy_len] = 0;

    handle_feature_command(usb.feature_last_command);
}

uint16_t build_feature_report(uint8_t *buffer, uint16_t reqlen) {
    if (!buffer || reqlen == 0) {
        return 0;
    }

    usb.feature_get_count++;
    if (usb.feature_reply_len == 0) {
        queue_feature_json("{\"ok\":true,\"profile\":\"ns2pro\",\"ready\":true}");
    }

    memset(buffer, 0, reqlen);
    if (reqlen < 11) {
        return reqlen;
    }

    memcpy(buffer, FEATURE_REPLY_MAGIC, strlen(FEATURE_REPLY_MAGIC));
    const uint16_t total = usb.feature_reply_len;
    uint16_t offset = usb.feature_reply_offset;
    if (offset > total) {
        offset = total;
        usb.feature_reply_offset = total;
    }

    const uint16_t remaining = static_cast<uint16_t>(total - offset);
    const uint16_t chunk_max = static_cast<uint16_t>(reqlen - 11);
    const uint16_t chunk_len = remaining < chunk_max ? remaining : chunk_max;

    buffer[6] = static_cast<uint8_t>(total & 0xff);
    buffer[7] = static_cast<uint8_t>((total >> 8) & 0xff);
    buffer[8] = static_cast<uint8_t>(offset & 0xff);
    buffer[9] = static_cast<uint8_t>((offset >> 8) & 0xff);
    buffer[10] = static_cast<uint8_t>(chunk_len);

    if (chunk_len > 0) {
        memcpy(buffer + 11, usb.feature_reply + offset, chunk_len);
        usb.feature_reply_offset = static_cast<uint16_t>(offset + chunk_len);
        if (usb.feature_reply_offset >= total) {
            usb.feature_reply_complete = true;
            if (!usb.bootrom_requested) {
                usb.feature_reply_len = 0;
                usb.feature_reply_offset = 0;
            }
        }
    }

    return reqlen;
}

} // namespace

void ns2_usb_init() {
    memset(&usb, 0, sizeof(usb));
    set_usb_report_rate(ns2_config_report_rate_hz());
    usb.hd_enabled = ns2_config_rumble_enabled();
    usb.hd_scale_percent = ns2_config_rumble_scale_percent();
    usb.hd_hold_us = static_cast<uint32_t>(ns2_config_rumble_hold_ms()) * 1000u;
    usb.hd_tick_us = static_cast<uint32_t>(ns2_config_rumble_tick_ms()) * 1000u;
    usb.hd_stop_packets = ns2_config_rumble_stop_packets();
    build_zero_ble_vibration(usb.hd_left_vibration);
    build_zero_ble_vibration(usb.hd_right_vibration);
    printf("[NS2 USB] Nintendo HID mode VID=057e PID=2069 report=0x05 rate=%uHz raw=%u\n",
           static_cast<unsigned>(usb.report_rate_hz),
           ns2_config_usb_raw_passthrough_enabled() ? 1u : 0u);
}

void ns2_usb_task() {
    flush_vendor_reply();
    hd_rumble_task();

    const uint64_t now = time_us_64();
    if (usb.next_report_us == 0 || now >= usb.next_report_us) {
        usb.next_report_us = now + usb.report_interval_us;
        send_report_now();
    }

    if (usb.bootrom_requested &&
        usb.feature_reply_complete) {
        sleep_ms(50);
        reset_usb_boot(0, 0);
    }
}

void ns2_usb_get_stats(Ns2UsbStats *out) {
    if (!out) {
        return;
    }
    memset(out, 0, sizeof(*out));
    out->mounted = usb.mounted;
    out->suspended = usb.suspended;
    out->reports_sent = usb.reports_sent;
    out->reports_failed = usb.reports_failed;
    out->raw_passthrough_reports = usb.raw_passthrough_reports;
    out->parsed_reports = usb.parsed_reports;
    out->report_rate_hz = usb.report_rate_hz;
    out->report_interval_us = usb.report_interval_us;
    out->hid_out_count = usb.hid_out_count;
    out->hid_last_report_id = usb.hid_last_report_id;
    out->hid_last_effective_report_id = usb.hid_last_effective_report_id;
    out->hid_last_type = usb.hid_last_type;
    out->hid_last_len = usb.hid_last_len;
    out->vendor_out_count = usb.vendor_out_count;
    out->vendor_in_count = usb.vendor_in_count;
    out->vendor_last_rx_len = usb.vendor_last_rx_len;
    out->vendor_last_tx_len = usb.vendor_last_tx_len;
    out->vendor_last_cmd = usb.vendor_last_cmd;
    out->vendor_last_arg = usb.vendor_last_arg;
    out->rumble_active = usb.hd_stream_active;
    out->rumble_updates = usb.hd_stream_updates;
    out->rumble_writes = usb.hd_stream_writes;
    out->rumble_stops = usb.hd_stream_stops;
    out->rumble_errors = usb.hd_stream_errors;
    out->rumble_enabled = usb.hd_enabled;
    out->rumble_scale_percent = usb.hd_scale_percent;
    out->rumble_hold_ms = static_cast<uint16_t>(usb.hd_hold_us / 1000);
    out->rumble_tick_ms = static_cast<uint16_t>(usb.hd_tick_us / 1000);
    out->rumble_stop_packets = usb.hd_stop_packets;
    out->feature_set_count = usb.feature_set_count;
    out->feature_get_count = usb.feature_get_count;
}

bool ns2_usb_handle_debug_command(const char *line, char *out, size_t out_len) {
    if (!line || !out || out_len == 0) {
        return false;
    }

    const char *command = skip_spaces(line);
    if (!command_has_prefix(command, "rumble") &&
        !command_has_prefix(command, "usb rate") &&
        !command_has_prefix(command, "usb raw") &&
        !command_has_prefix(command, "report rate") &&
        !command_is(command, "usb config") &&
        !command_is(command, "report config") &&
        !command_has_prefix(command, "settings") &&
        !command_has_prefix(command, "config") &&
        !command_has_prefix(command, "display") &&
        !command_has_prefix(command, "screen") &&
        !command_has_prefix(command, "web parse") &&
        !command_has_prefix(command, "webui parse")) {
        return false;
    }

    if (command_has_prefix(command, "rumble")) {
        handle_rumble_command(command);
    } else if (command_has_prefix(command, "settings") ||
               command_has_prefix(command, "config") ||
               command_has_prefix(command, "display") ||
               command_has_prefix(command, "screen") ||
               command_has_prefix(command, "web parse") ||
               command_has_prefix(command, "webui parse")) {
        handle_settings_command(command);
    } else {
        handle_usb_command(command);
    }
    const size_t copy_len = usb.feature_reply_len < out_len - 1 ?
        usb.feature_reply_len :
        out_len - 1;
    memcpy(out, usb.feature_reply, copy_len);
    out[copy_len] = 0;
    return true;
}

void tud_mount_cb(void) {
    usb.mounted = true;
    usb.suspended = false;
    printf("[NS2 USB] mounted\n");
}

void tud_umount_cb(void) {
    usb.mounted = false;
    usb.suspended = false;
    usb.pending_len = 0;
    usb.pending_offset = 0;
    printf("[NS2 USB] unmounted\n");
}

void tud_suspend_cb(bool remote_wakeup_en) {
    (void)remote_wakeup_en;
    usb.suspended = true;
    printf("[NS2 USB] suspended\n");
}

void tud_resume_cb(void) {
    usb.suspended = false;
    printf("[NS2 USB] resumed\n");
}

uint16_t tud_hid_get_report_cb(uint8_t instance,
                               uint8_t report_id,
                               hid_report_type_t report_type,
                               uint8_t *buffer,
                               uint16_t reqlen) {
    (void)instance;
    if (!buffer || reqlen == 0) {
        return 0;
    }

    if (report_type == HID_REPORT_TYPE_FEATURE &&
        (report_id == 0 || report_id == NS2_USB_MANAGER_FEATURE_REPORT_ID)) {
        return build_feature_report(buffer, reqlen);
    }

    if (report_type == HID_REPORT_TYPE_INPUT &&
        (report_id == 0 || report_id == NS2_USB_NINTENDO_INPUT_REPORT_ID)) {
        uint8_t report[NS2_USB_NINTENDO_REPORT_SIZE];
        make_neutral_report(report);
        const uint16_t offset = report_id == NS2_USB_NINTENDO_INPUT_REPORT_ID ? 1 : 0;
        uint16_t len = static_cast<uint16_t>(NS2_USB_NINTENDO_REPORT_SIZE - offset);
        if (len > reqlen) {
            len = reqlen;
        }
        memcpy(buffer, report + offset, len);
        return len;
    }

    memset(buffer, 0, reqlen);
    return reqlen;
}

void tud_hid_set_report_cb(uint8_t instance,
                           uint8_t report_id,
                           hid_report_type_t report_type,
                           uint8_t const *buffer,
                           uint16_t bufsize) {
    uint8_t effective_report_id = report_id;
    uint8_t const *payload = buffer;
    uint16_t payload_size = bufsize;

    if (effective_report_id == 0 && buffer && bufsize > 0) {
        effective_report_id = buffer[0];
        payload = buffer + 1;
        payload_size = static_cast<uint16_t>(bufsize - 1);
    }

    usb.hid_last_report_id = report_id;
    usb.hid_last_effective_report_id = effective_report_id;
    usb.hid_last_type = static_cast<uint8_t>(report_type);
    usb.hid_last_len = bufsize;
    usb.hid_last_first_byte = buffer && bufsize > 0 ? buffer[0] : 0;

    if (report_type == HID_REPORT_TYPE_FEATURE &&
        effective_report_id == NS2_USB_MANAGER_FEATURE_REPORT_ID) {
        receive_feature_command(payload, payload_size);
        return;
    }

    usb.hid_out_count++;

    if (effective_report_id == NS2_USB_NINTENDO_OUTPUT_REPORT_ID) {
        uint8_t full_report[NS2_USB_NINTENDO_REPORT_SIZE];
        memset(full_report, 0, sizeof(full_report));
        full_report[0] = effective_report_id;
        const uint16_t copy_len = payload_size > NS2_USB_NINTENDO_REPORT_SIZE - 1 ?
            NS2_USB_NINTENDO_REPORT_SIZE - 1 :
            payload_size;
        if (payload && copy_len > 0) {
            memcpy(full_report + 1, payload, copy_len);
        }
        bridge_hid_output_to_ble(full_report, static_cast<uint16_t>(copy_len + 1));

        printf("[NS2 USB] HID OUT report=0x%02x type=%u len=%u first=%02x\n",
               effective_report_id,
               static_cast<unsigned>(report_type),
               static_cast<unsigned>(payload_size),
               payload && payload_size > 0 ? payload[0] : 0);
    }
}

void tud_vendor_rx_cb(uint8_t itf, uint8_t const *buffer, uint16_t bufsize) {
    if (itf != VENDOR_INTERFACE || !buffer || bufsize == 0) {
        return;
    }

    uint8_t cmd[VENDOR_REPLY_MAX];
    const uint16_t cmd_len = bufsize > sizeof(cmd) ? sizeof(cmd) : bufsize;
    memcpy(cmd, buffer, cmd_len);
    tud_vendor_n_read_flush(itf);

    usb.vendor_out_count++;
    usb.vendor_last_rx_len = cmd_len;
    usb.vendor_last_cmd = cmd[0];
    usb.vendor_last_arg = cmd_len > 3 ? cmd[3] : 0;

    uint8_t reply[VENDOR_REPLY_MAX];
    const size_t reply_len = build_vendor_reply(cmd, cmd_len, reply, sizeof(reply));
    if (reply_len > 0) {
        queue_vendor_reply(itf, reply, reply_len);
    }
}

void tud_vendor_tx_cb(uint8_t itf, uint32_t sent_bytes) {
    (void)sent_bytes;
    usb.vendor_in_done_count++;
    if (itf == usb.pending_itf) {
        flush_vendor_reply();
    }
}
