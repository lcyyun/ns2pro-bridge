#pragma once

#include "esp_err.h"

#include <cstdint>

namespace ns2 {

enum class OutputMode : uint8_t {
    Nintendo = 0,
    XInput = 1,
    DualSense = 2,
};

struct RuntimeConfig {
    bool auto_connect = true;
    bool saved_target_valid = false;
    uint8_t saved_addr[6] = {};
    uint8_t saved_addr_type = 0;
    bool raw_passthrough = false;
    bool web_parse_reports = true;
    bool rumble_enabled = true;
    uint16_t rumble_scale_percent = 60;
    uint16_t rumble_hold_ms = 140;
    uint16_t rumble_tick_ms = 12;
    uint8_t rumble_stop_packets = 3;
    uint16_t report_rate_hz = 250;
    OutputMode output_mode = OutputMode::Nintendo;
};

esp_err_t config_init();
esp_err_t config_save();
void config_get(RuntimeConfig *out);

bool config_auto_connect();
void config_set_auto_connect(bool enabled);

bool config_has_saved_target();
void config_get_saved_target(uint8_t addr[6], uint8_t *addr_type);
void config_set_saved_target(const uint8_t addr[6], uint8_t addr_type);
void config_forget_saved_target();

void config_set_usb(bool raw_passthrough,
                    bool web_parse_reports,
                    bool rumble_enabled,
                    uint16_t rumble_scale_percent,
                    uint16_t rumble_hold_ms,
                    uint16_t rumble_tick_ms,
                    uint8_t rumble_stop_packets,
                    uint16_t report_rate_hz,
                    OutputMode output_mode);

OutputMode config_output_mode();
void config_set_output_mode(OutputMode mode);
const char *config_output_mode_name(OutputMode mode);

} // namespace ns2
