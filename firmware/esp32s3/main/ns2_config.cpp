#include "ns2_config.h"

#include <cstring>

#include "esp_check.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"

namespace ns2 {
namespace {

const char *TAG = "ns2-config";
constexpr char kNamespace[] = "ns2";
constexpr char kBlobKey[] = "cfg";
constexpr uint32_t kMagic = 0x3253334e; // NS32
constexpr uint16_t kVersion = 2;

struct StoredConfig {
    uint32_t magic = kMagic;
    uint16_t version = kVersion;
    uint8_t auto_connect = 1;
    uint8_t saved_target_valid = 0;
    uint8_t saved_addr[6] = {};
    uint8_t saved_addr_type = 0;
    uint8_t raw_passthrough = 0;
    uint8_t web_parse_reports = 1;
    uint8_t rumble_enabled = 1;
    uint16_t rumble_scale_percent = 60;
    uint16_t rumble_hold_ms = 140;
    uint16_t rumble_tick_ms = 12;
    uint8_t rumble_stop_packets = 3;
    uint16_t report_rate_hz = 250;
    uint8_t output_mode = static_cast<uint8_t>(OutputMode::Nintendo);
};

RuntimeConfig s_config;
bool s_ready = false;

StoredConfig to_stored() {
    StoredConfig stored;
    stored.auto_connect = s_config.auto_connect ? 1 : 0;
    stored.saved_target_valid = s_config.saved_target_valid ? 1 : 0;
    std::memcpy(stored.saved_addr, s_config.saved_addr, sizeof(stored.saved_addr));
    stored.saved_addr_type = s_config.saved_addr_type;
    stored.raw_passthrough = s_config.raw_passthrough ? 1 : 0;
    stored.web_parse_reports = s_config.web_parse_reports ? 1 : 0;
    stored.rumble_enabled = s_config.rumble_enabled ? 1 : 0;
    stored.rumble_scale_percent = s_config.rumble_scale_percent == 0 ? 60 : s_config.rumble_scale_percent;
    stored.rumble_hold_ms = s_config.rumble_hold_ms == 0 ? 140 : s_config.rumble_hold_ms;
    stored.rumble_tick_ms = s_config.rumble_tick_ms == 0 ? 12 : s_config.rumble_tick_ms;
    stored.rumble_stop_packets = s_config.rumble_stop_packets == 0 ? 3 : s_config.rumble_stop_packets;
    stored.report_rate_hz = s_config.report_rate_hz == 0 ? 250 : s_config.report_rate_hz;
    stored.output_mode = static_cast<uint8_t>(s_config.output_mode);
    return stored;
}

OutputMode sanitize_output_mode(uint8_t mode) {
    switch (static_cast<OutputMode>(mode)) {
    case OutputMode::Nintendo:
    case OutputMode::XInput:
    case OutputMode::DualSense:
        return static_cast<OutputMode>(mode);
    default:
        return OutputMode::Nintendo;
    }
}

void from_stored(const StoredConfig &stored) {
    s_config.auto_connect = stored.auto_connect != 0;
    s_config.saved_target_valid = stored.saved_target_valid != 0;
    std::memcpy(s_config.saved_addr, stored.saved_addr, sizeof(s_config.saved_addr));
    s_config.saved_addr_type = stored.saved_addr_type;
    s_config.raw_passthrough = stored.raw_passthrough != 0;
    s_config.web_parse_reports = stored.web_parse_reports != 0;
    s_config.rumble_enabled = stored.rumble_enabled != 0;
    s_config.rumble_scale_percent = stored.rumble_scale_percent == 0 ? 60 : stored.rumble_scale_percent;
    s_config.rumble_hold_ms = stored.rumble_hold_ms == 0 ? 140 : stored.rumble_hold_ms;
    s_config.rumble_tick_ms = stored.rumble_tick_ms == 0 ? 12 : stored.rumble_tick_ms;
    s_config.rumble_stop_packets = stored.rumble_stop_packets == 0 ? 3 : stored.rumble_stop_packets;
    s_config.report_rate_hz = stored.report_rate_hz == 0 ? 250 : stored.report_rate_hz;
    s_config.output_mode = sanitize_output_mode(stored.output_mode);
}

} // namespace

esp_err_t config_init() {
    if (s_ready) {
        return ESP_OK;
    }
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        return err;
    }

    nvs_handle_t handle = 0;
    err = nvs_open(kNamespace, NVS_READONLY, &handle);
    if (err == ESP_OK) {
        StoredConfig stored;
        size_t len = sizeof(stored);
        err = nvs_get_blob(handle, kBlobKey, &stored, &len);
        nvs_close(handle);
        if (err == ESP_OK && len == sizeof(stored) && stored.magic == kMagic && stored.version == kVersion) {
            from_stored(stored);
        }
    }
    s_ready = true;
    ESP_LOGI(TAG, "config loaded auto=%u saved=%u report=%u rumble=%u mode=%s",
             s_config.auto_connect ? 1u : 0u,
             s_config.saved_target_valid ? 1u : 0u,
             static_cast<unsigned>(s_config.report_rate_hz),
             s_config.rumble_enabled ? 1u : 0u,
             config_output_mode_name(s_config.output_mode));
    return ESP_OK;
}

esp_err_t config_save() {
    ESP_RETURN_ON_ERROR(config_init(), TAG, "config init failed");
    nvs_handle_t handle = 0;
    esp_err_t err = nvs_open(kNamespace, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return err;
    }
    const StoredConfig stored = to_stored();
    err = nvs_set_blob(handle, kBlobKey, &stored, sizeof(stored));
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    return err;
}

void config_get(RuntimeConfig *out) {
    if (out != nullptr) {
        *out = s_config;
    }
}

bool config_auto_connect() {
    return s_config.auto_connect;
}

void config_set_auto_connect(bool enabled) {
    s_config.auto_connect = enabled;
}

bool config_has_saved_target() {
    return s_config.saved_target_valid;
}

void config_get_saved_target(uint8_t addr[6], uint8_t *addr_type) {
    if (addr != nullptr) {
        std::memcpy(addr, s_config.saved_addr, 6);
    }
    if (addr_type != nullptr) {
        *addr_type = s_config.saved_addr_type;
    }
}

void config_set_saved_target(const uint8_t addr[6], uint8_t addr_type) {
    if (addr == nullptr) {
        return;
    }
    std::memcpy(s_config.saved_addr, addr, 6);
    s_config.saved_addr_type = addr_type;
    s_config.saved_target_valid = true;
}

void config_forget_saved_target() {
    s_config.saved_target_valid = false;
    std::memset(s_config.saved_addr, 0, sizeof(s_config.saved_addr));
    s_config.saved_addr_type = 0;
}

void config_set_usb(bool raw_passthrough,
                    bool web_parse_reports,
                    bool rumble_enabled,
                    uint16_t rumble_scale_percent,
                    uint16_t rumble_hold_ms,
                    uint16_t rumble_tick_ms,
                    uint8_t rumble_stop_packets,
                    uint16_t report_rate_hz,
                    OutputMode output_mode) {
    s_config.raw_passthrough = raw_passthrough;
    s_config.web_parse_reports = web_parse_reports;
    s_config.rumble_enabled = rumble_enabled;
    s_config.rumble_scale_percent = rumble_scale_percent == 0 ? 60 : rumble_scale_percent;
    s_config.rumble_hold_ms = rumble_hold_ms == 0 ? 140 : rumble_hold_ms;
    s_config.rumble_tick_ms = rumble_tick_ms == 0 ? 12 : rumble_tick_ms;
    s_config.rumble_stop_packets = rumble_stop_packets == 0 ? 3 : rumble_stop_packets;
    s_config.report_rate_hz = report_rate_hz == 0 ? 250 : report_rate_hz;
    s_config.output_mode = output_mode;
}

OutputMode config_output_mode() {
    return s_config.output_mode;
}

void config_set_output_mode(OutputMode mode) {
    s_config.output_mode = mode;
}

const char *config_output_mode_name(OutputMode mode) {
    switch (mode) {
    case OutputMode::Nintendo:
        return "nintendo";
    case OutputMode::XInput:
        return "xinput";
    case OutputMode::DualSense:
        return "dualsense";
    default:
        return "nintendo";
    }
}

} // namespace ns2
