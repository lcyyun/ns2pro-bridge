#include "ns2_state.h"

#include <cstdio>
#include <cstring>

#include "hardware/flash.h"
#include "hardware/sync.h"
#include "pico/platform.h"
#include "utils.h"

namespace {

constexpr uint32_t NS2_CONFIG_MAGIC = 0x32534e50; // "PNS2" little-endian
constexpr uint16_t NS2_CONFIG_VERSION = 1;
constexpr uint32_t NS2_CONFIG_FLASH_OFFSET = PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE;
constexpr uint8_t CONFIG_FLAG_DISPLAY_DISABLED = 0x01;
constexpr uint8_t CONFIG_FLAG_USB_RAW_DISABLED = 0x02;
constexpr uint8_t CONFIG_FLAG_WEB_PARSE_DISABLED = 0x04;
constexpr uint8_t CONFIG_FLAG_RUMBLE_DISABLED = 0x08;
constexpr uint16_t CONFIG_DEFAULT_REPORT_RATE_HZ = 250;
constexpr uint16_t CONFIG_DEFAULT_RUMBLE_SCALE_PERCENT = 60;
constexpr uint16_t CONFIG_DEFAULT_RUMBLE_HOLD_MS = 140;
constexpr uint16_t CONFIG_DEFAULT_RUMBLE_TICK_MS = 30;
constexpr uint8_t CONFIG_DEFAULT_RUMBLE_STOP_PACKETS = 3;

Ns2ConfigBlock config{};

static_assert(sizeof(Ns2ConfigBlock) <= FLASH_PAGE_SIZE);
static_assert(NS2_CONFIG_FLASH_OFFSET % FLASH_SECTOR_SIZE == 0);

const Ns2ConfigBlock *flash_config() {
    return reinterpret_cast<const Ns2ConfigBlock *>(XIP_BASE + NS2_CONFIG_FLASH_OFFSET);
}

uint32_t calc_body_crc(const Ns2ConfigBlock &block) {
    return crc32(reinterpret_cast<const uint8_t *>(&block.body), sizeof(Ns2ConfigBody));
}

bool config_crc_valid(const Ns2ConfigBlock &block) {
    return block.crc32 == calc_body_crc(block);
}

uint16_t clamp_u16(uint16_t value, uint16_t fallback, uint16_t min_value, uint16_t max_value) {
    if (value == 0) {
        return fallback;
    }
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

uint8_t clamp_u8(uint8_t value, uint8_t fallback, uint8_t min_value, uint8_t max_value) {
    if (value == 0) {
        return fallback;
    }
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

void validate_config() {
    bool reset = false;
    if (config.magic != NS2_CONFIG_MAGIC || config.version != NS2_CONFIG_VERSION) {
        reset = true;
    }
    if (!reset && !config_crc_valid(config)) {
        reset = true;
    }

    if (reset) {
        ns2_config_reset_defaults();
        return;
    }

    if (config.body.auto_connect_enabled > 1) {
        config.body.auto_connect_enabled = 1;
    }
    if (config.body.saved_addr_type > 1) {
        config.body.saved_addr_type = 0;
    }
    if (config.body.scan_policy > static_cast<uint8_t>(Ns2ScanPolicy::SavedFirstThenAnyCandidate)) {
        config.body.scan_policy = static_cast<uint8_t>(Ns2ScanPolicy::SavedFirstThenAnyCandidate);
    }
    config.body.runtime_flags &= CONFIG_FLAG_DISPLAY_DISABLED |
                                 CONFIG_FLAG_USB_RAW_DISABLED |
                                 CONFIG_FLAG_WEB_PARSE_DISABLED |
                                 CONFIG_FLAG_RUMBLE_DISABLED;
    config.body.report_rate_hz = clamp_u16(config.body.report_rate_hz,
                                           CONFIG_DEFAULT_REPORT_RATE_HZ,
                                           60,
                                           1000);
    config.body.rumble_scale_percent = clamp_u16(config.body.rumble_scale_percent,
                                                 CONFIG_DEFAULT_RUMBLE_SCALE_PERCENT,
                                                 5,
                                                 250);
    config.body.rumble_hold_ms = clamp_u16(config.body.rumble_hold_ms,
                                           CONFIG_DEFAULT_RUMBLE_HOLD_MS,
                                           20,
                                           3000);
    config.body.rumble_tick_ms = clamp_u16(config.body.rumble_tick_ms,
                                           CONFIG_DEFAULT_RUMBLE_TICK_MS,
                                           5,
                                           100);
    config.body.rumble_stop_packets = clamp_u8(config.body.rumble_stop_packets,
                                               CONFIG_DEFAULT_RUMBLE_STOP_PACKETS,
                                               1,
                                               12);
}

} // namespace

const char *ns2_ble_state_name(Ns2BleState state) {
    switch (state) {
        case Ns2BleState::Boot:
            return "boot";
        case Ns2BleState::BleInit:
            return "ble_init";
        case Ns2BleState::Idle:
            return "idle";
        case Ns2BleState::Scanning:
            return "scanning";
        case Ns2BleState::Connecting:
            return "connecting";
        case Ns2BleState::Pairing:
            return "pairing";
        case Ns2BleState::Discovering:
            return "discovering";
        case Ns2BleState::Subscribing:
            return "subscribing";
        case Ns2BleState::InitializingController:
            return "initializing_controller";
        case Ns2BleState::ConnectedNoNotify:
            return "connected_no_notify";
        case Ns2BleState::ConnectedLive:
            return "connected_live";
        case Ns2BleState::Disconnected:
            return "disconnected";
        case Ns2BleState::Backoff:
            return "backoff";
        case Ns2BleState::Error:
            return "error";
        default:
            return "unknown";
    }
}

void ns2_config_reset_defaults() {
    memset(&config, 0, sizeof(config));
    config.magic = NS2_CONFIG_MAGIC;
    config.version = NS2_CONFIG_VERSION;
    config.body.auto_connect_enabled = 1;
    config.body.saved_addr_type = 0;
    config.body.last_success_boot_counter = 0;
    config.body.scan_policy = static_cast<uint8_t>(Ns2ScanPolicy::SavedFirstThenAnyCandidate);
    config.body.runtime_flags = CONFIG_FLAG_DISPLAY_DISABLED |
                                CONFIG_FLAG_USB_RAW_DISABLED;
    config.body.report_rate_hz = CONFIG_DEFAULT_REPORT_RATE_HZ;
    config.body.rumble_scale_percent = CONFIG_DEFAULT_RUMBLE_SCALE_PERCENT;
    config.body.rumble_hold_ms = CONFIG_DEFAULT_RUMBLE_HOLD_MS;
    config.body.rumble_tick_ms = CONFIG_DEFAULT_RUMBLE_TICK_MS;
    config.body.rumble_stop_packets = CONFIG_DEFAULT_RUMBLE_STOP_PACKETS;
    config.crc32 = calc_body_crc(config);
}

void ns2_config_load() {
    memcpy(&config, flash_config(), sizeof(config));
    validate_config();
    config.body.auto_connect_enabled = 1;
}

bool ns2_config_save() {
    config.magic = NS2_CONFIG_MAGIC;
    config.version = NS2_CONFIG_VERSION;
    config.crc32 = calc_body_crc(config);

    alignas(4) uint8_t page[FLASH_PAGE_SIZE];
    memset(page, 0xff, sizeof(page));
    memcpy(page, &config, sizeof(config));

    const uint32_t interrupts = save_and_disable_interrupts();
    flash_range_erase(NS2_CONFIG_FLASH_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(NS2_CONFIG_FLASH_OFFSET, page, sizeof(page));
    restore_interrupts(interrupts);

    Ns2ConfigBlock verify{};
    memcpy(&verify, flash_config(), sizeof(verify));
    const bool ok = verify.magic == NS2_CONFIG_MAGIC &&
                    verify.version == NS2_CONFIG_VERSION &&
                    verify.crc32 == calc_body_crc(verify);
    printf("[NS2 Config] flash verify %s\n", ok ? "success" : "failed");
    return ok;
}

const Ns2ConfigBody &ns2_config_get() {
    return config.body;
}

bool ns2_config_auto_connect_enabled() {
    return config.body.auto_connect_enabled != 0;
}

void ns2_config_set_auto_connect(bool enabled) {
    config.body.auto_connect_enabled = enabled ? 1 : 0;
}

bool ns2_config_display_enabled() {
    return (config.body.runtime_flags & CONFIG_FLAG_DISPLAY_DISABLED) == 0;
}

void ns2_config_set_display_enabled(bool enabled) {
    if (enabled) {
        config.body.runtime_flags &= static_cast<uint8_t>(~CONFIG_FLAG_DISPLAY_DISABLED);
    } else {
        config.body.runtime_flags |= CONFIG_FLAG_DISPLAY_DISABLED;
    }
}

bool ns2_config_usb_raw_passthrough_enabled() {
    return (config.body.runtime_flags & CONFIG_FLAG_USB_RAW_DISABLED) == 0;
}

void ns2_config_set_usb_raw_passthrough_enabled(bool enabled) {
    if (enabled) {
        config.body.runtime_flags &= static_cast<uint8_t>(~CONFIG_FLAG_USB_RAW_DISABLED);
    } else {
        config.body.runtime_flags |= CONFIG_FLAG_USB_RAW_DISABLED;
    }
}

bool ns2_config_web_parse_reports_enabled() {
    return (config.body.runtime_flags & CONFIG_FLAG_WEB_PARSE_DISABLED) == 0;
}

void ns2_config_set_web_parse_reports_enabled(bool enabled) {
    if (enabled) {
        config.body.runtime_flags &= static_cast<uint8_t>(~CONFIG_FLAG_WEB_PARSE_DISABLED);
    } else {
        config.body.runtime_flags |= CONFIG_FLAG_WEB_PARSE_DISABLED;
    }
}

bool ns2_config_rumble_enabled() {
    return (config.body.runtime_flags & CONFIG_FLAG_RUMBLE_DISABLED) == 0;
}

void ns2_config_set_rumble_enabled(bool enabled) {
    if (enabled) {
        config.body.runtime_flags &= static_cast<uint8_t>(~CONFIG_FLAG_RUMBLE_DISABLED);
    } else {
        config.body.runtime_flags |= CONFIG_FLAG_RUMBLE_DISABLED;
    }
}

uint16_t ns2_config_report_rate_hz() {
    return config.body.report_rate_hz == 0 ? CONFIG_DEFAULT_REPORT_RATE_HZ : config.body.report_rate_hz;
}

void ns2_config_set_report_rate_hz(uint16_t rate_hz) {
    config.body.report_rate_hz = clamp_u16(rate_hz, CONFIG_DEFAULT_REPORT_RATE_HZ, 60, 1000);
}

uint16_t ns2_config_rumble_scale_percent() {
    return config.body.rumble_scale_percent == 0 ?
        CONFIG_DEFAULT_RUMBLE_SCALE_PERCENT :
        config.body.rumble_scale_percent;
}

uint16_t ns2_config_rumble_hold_ms() {
    return config.body.rumble_hold_ms == 0 ?
        CONFIG_DEFAULT_RUMBLE_HOLD_MS :
        config.body.rumble_hold_ms;
}

uint16_t ns2_config_rumble_tick_ms() {
    return config.body.rumble_tick_ms == 0 ?
        CONFIG_DEFAULT_RUMBLE_TICK_MS :
        config.body.rumble_tick_ms;
}

uint8_t ns2_config_rumble_stop_packets() {
    return config.body.rumble_stop_packets == 0 ?
        CONFIG_DEFAULT_RUMBLE_STOP_PACKETS :
        config.body.rumble_stop_packets;
}

void ns2_config_set_rumble_tune(uint16_t scale_percent,
                                uint16_t hold_ms,
                                uint16_t tick_ms,
                                uint8_t stop_packets) {
    config.body.rumble_scale_percent = clamp_u16(scale_percent,
                                                 CONFIG_DEFAULT_RUMBLE_SCALE_PERCENT,
                                                 5,
                                                 250);
    config.body.rumble_hold_ms = clamp_u16(hold_ms,
                                           CONFIG_DEFAULT_RUMBLE_HOLD_MS,
                                           20,
                                           3000);
    config.body.rumble_tick_ms = clamp_u16(tick_ms,
                                           CONFIG_DEFAULT_RUMBLE_TICK_MS,
                                           5,
                                           100);
    config.body.rumble_stop_packets = clamp_u8(stop_packets,
                                               CONFIG_DEFAULT_RUMBLE_STOP_PACKETS,
                                               1,
                                               12);
}

bool ns2_addr_is_empty(const uint8_t addr[6]) {
    bool all_zero = true;
    bool all_ff = true;
    for (size_t i = 0; i < 6; i++) {
        all_zero = all_zero && addr[i] == 0x00;
        all_ff = all_ff && addr[i] == 0xff;
    }
    return all_zero || all_ff;
}

bool ns2_config_has_saved_target() {
    return !ns2_addr_is_empty(config.body.saved_addr);
}

void ns2_config_get_saved_target(uint8_t out_addr[6], uint8_t *out_addr_type) {
    memcpy(out_addr, config.body.saved_addr, 6);
    if (out_addr_type) {
        *out_addr_type = config.body.saved_addr_type;
    }
}

void ns2_config_set_saved_target(const uint8_t addr[6], uint8_t addr_type, uint32_t boot_counter) {
    memcpy(config.body.saved_addr, addr, 6);
    config.body.saved_addr_type = addr_type > 1 ? 0 : addr_type;
    config.body.last_success_boot_counter = boot_counter;
}

void ns2_config_forget_saved_target() {
    memset(config.body.saved_addr, 0, sizeof(config.body.saved_addr));
    config.body.saved_addr_type = 0;
    config.body.last_success_boot_counter = 0;
}

void ns2_format_addr(const uint8_t addr[6], uint8_t addr_type, char *out, size_t out_len) {
    if (!out || out_len == 0) {
        return;
    }
    snprintf(out, out_len, "%02x:%02x:%02x:%02x:%02x:%02x/%u",
             addr[0], addr[1], addr[2], addr[3], addr[4], addr[5],
             static_cast<unsigned>(addr_type));
}

bool ns2_addr_equal(const uint8_t a[6], const uint8_t b[6]) {
    return memcmp(a, b, 6) == 0;
}
