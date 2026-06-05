#ifndef DS5_BRIDGE_NS2_STATE_H
#define DS5_BRIDGE_NS2_STATE_H

#include <cstddef>
#include <cstdint>

enum class Ns2BleState : uint8_t {
    Boot = 0,
    BleInit,
    Idle,
    Scanning,
    Connecting,
    Pairing,
    Discovering,
    Subscribing,
    InitializingController,
    ConnectedNoNotify,
    ConnectedLive,
    Disconnected,
    Backoff,
    Error
};

enum class Ns2ScanPolicy : uint8_t {
    SavedFirstThenAnyCandidate = 0
};

struct __attribute__((packed)) Ns2ConfigBody {
    uint8_t auto_connect_enabled;
    uint8_t saved_addr[6];
    uint8_t saved_addr_type;
    uint32_t last_success_boot_counter;
    uint8_t scan_policy;
    uint8_t runtime_flags;
    uint16_t report_rate_hz;
    uint16_t rumble_scale_percent;
    uint16_t rumble_hold_ms;
    uint16_t rumble_tick_ms;
    uint8_t rumble_stop_packets;
    uint8_t reserved[5];
};

struct __attribute__((packed)) Ns2ConfigBlock {
    uint32_t magic;
    uint16_t version;
    uint32_t crc32;
    Ns2ConfigBody body;
};

const char *ns2_ble_state_name(Ns2BleState state);

void ns2_config_load();
bool ns2_config_save();
void ns2_config_reset_defaults();

const Ns2ConfigBody &ns2_config_get();
bool ns2_config_auto_connect_enabled();
void ns2_config_set_auto_connect(bool enabled);

bool ns2_config_display_enabled();
void ns2_config_set_display_enabled(bool enabled);
bool ns2_config_usb_raw_passthrough_enabled();
void ns2_config_set_usb_raw_passthrough_enabled(bool enabled);
bool ns2_config_web_parse_reports_enabled();
void ns2_config_set_web_parse_reports_enabled(bool enabled);
bool ns2_config_rumble_enabled();
void ns2_config_set_rumble_enabled(bool enabled);
uint16_t ns2_config_report_rate_hz();
void ns2_config_set_report_rate_hz(uint16_t rate_hz);
uint16_t ns2_config_rumble_scale_percent();
uint16_t ns2_config_rumble_hold_ms();
uint16_t ns2_config_rumble_tick_ms();
uint8_t ns2_config_rumble_stop_packets();
void ns2_config_set_rumble_tune(uint16_t scale_percent,
                                uint16_t hold_ms,
                                uint16_t tick_ms,
                                uint8_t stop_packets);

bool ns2_config_has_saved_target();
void ns2_config_get_saved_target(uint8_t out_addr[6], uint8_t *out_addr_type);
void ns2_config_set_saved_target(const uint8_t addr[6], uint8_t addr_type, uint32_t boot_counter);
void ns2_config_forget_saved_target();

void ns2_format_addr(const uint8_t addr[6], uint8_t addr_type, char *out, size_t out_len);
bool ns2_addr_is_empty(const uint8_t addr[6]);
bool ns2_addr_equal(const uint8_t a[6], const uint8_t b[6]);

#endif
