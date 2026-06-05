#ifndef DS5_BRIDGE_NS2_INPUT_H
#define DS5_BRIDGE_NS2_INPUT_H

#include <cstddef>
#include <cstdint>

constexpr size_t NS2_MOTION_SAMPLE_SIZE = 12;
constexpr size_t NS2_INPUT_RAW_REPORT_MAX = 64;

enum class Ns2InputReportKind : uint8_t {
    Unknown = 0,
    Fd2,
    Legacy
};

struct Ns2InputSnapshot {
    bool valid;
    Ns2InputReportKind kind;
    uint16_t len;
    uint8_t first_byte;
    uint32_t updates;
    uint32_t parse_errors;
    uint32_t buttons;
    uint16_t lx;
    uint16_t ly;
    uint16_t rx;
    uint16_t ry;
    bool raw_valid;
    uint16_t raw_len;
    uint8_t raw[NS2_INPUT_RAW_REPORT_MAX];
    bool motion_valid;
    uint32_t motion_updates;
    Ns2InputReportKind motion_kind;
    uint16_t motion_len;
    uint8_t motion_offset;
    uint8_t motion[NS2_MOTION_SAMPLE_SIZE];
};

struct Ns2MotionSample {
    bool valid;
    int16_t accel[3];
    int16_t gyro[3];
    uint8_t raw[NS2_MOTION_SAMPLE_SIZE];
};

void ns2_input_reset();
bool ns2_input_parse_notify(Ns2InputReportKind kind, const uint8_t *data, uint16_t len);
bool ns2_input_get_snapshot(Ns2InputSnapshot *out);
bool ns2_input_get_motion_sample(Ns2MotionSample *out);
const char *ns2_input_kind_name(Ns2InputReportKind kind);
void ns2_input_format_buttons(uint32_t buttons, char *out, size_t out_len);
void ns2_input_format_motion_json(const Ns2InputSnapshot *input, char *out, size_t out_len);

#endif
