#include "ns2_status.h"

#include <cstdio>
#include <cstring>

#include "ble/le_device_db.h"
#include "pico/cyw43_arch.h"
#include "pico/time.h"

namespace {

constexpr size_t MAX_CANDIDATES = 8;
constexpr uint64_t LIVE_NOTIFY_STALE_US = 1'000'000;
constexpr uint64_t RATE_WINDOW_US = 1'000'000;

struct RuntimeStatus {
    Ns2BleState state;
    Ns2Candidate candidates[MAX_CANDIDATES];
    uint32_t candidate_seen;
    uint32_t connect_attempts;
    uint32_t disconnect_count;
    uint8_t last_disconnect_reason;
    bool local_addr_valid;
    uint8_t local_addr[6];
    bool connected_addr_valid;
    uint8_t connected_addr[6];
    uint8_t connected_addr_type;
    int8_t rssi;
    uint32_t notify_count;
    uint32_t notify_hz;
    uint32_t rate_window_start_count;
    uint64_t rate_window_start_us;
    uint64_t last_notify_us;
    uint16_t last_notify_len;
    char last_error[64];
};

RuntimeStatus status{};
bool led_state = false;
uint64_t led_epoch_us = 0;

void set_led(bool on) {
    if (led_state == on) {
        return;
    }
    led_state = on;
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, on);
}

bool pulse_train(uint64_t elapsed_us, uint32_t pulses) {
    constexpr uint64_t SLOT_US = 160'000;
    constexpr uint64_t PERIOD_US = 1'200'000;
    const uint64_t phase = elapsed_us % PERIOD_US;
    for (uint32_t i = 0; i < pulses; i++) {
        const uint64_t start = i * SLOT_US * 2;
        if (phase >= start && phase < start + SLOT_US) {
            return true;
        }
    }
    return false;
}

void json_escape(const char *in, char *out, size_t out_len) {
    if (!out || out_len == 0) {
        return;
    }
    size_t used = 0;
    for (const char *p = in ? in : ""; *p && used + 1 < out_len; p++) {
        const unsigned char ch = static_cast<unsigned char>(*p);
        if ((ch == '"' || ch == '\\') && used + 2 < out_len) {
            out[used++] = '\\';
            out[used++] = static_cast<char>(ch);
        } else if (ch >= 0x20) {
            out[used++] = static_cast<char>(ch);
        }
    }
    out[used] = 0;
}

void update_notify_hz(uint64_t now_us) {
    if (status.rate_window_start_us == 0) {
        status.rate_window_start_us = now_us;
        status.rate_window_start_count = status.notify_count;
        return;
    }
    const uint64_t elapsed = now_us - status.rate_window_start_us;
    if (elapsed >= RATE_WINDOW_US) {
        const uint32_t delta = status.notify_count - status.rate_window_start_count;
        status.notify_hz = static_cast<uint32_t>((static_cast<uint64_t>(delta) * 1'000'000ULL) / elapsed);
        status.rate_window_start_us = now_us;
        status.rate_window_start_count = status.notify_count;
    }
}

} // namespace

void ns2_status_init() {
    memset(&status, 0, sizeof(status));
    status.state = Ns2BleState::Boot;
    status.rssi = 0;
    led_state = false;
    led_epoch_us = time_us_64();
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, false);
}

void ns2_status_set_state(Ns2BleState state) {
    if (status.state != state) {
        status.state = state;
        led_epoch_us = time_us_64();
    }
}

Ns2BleState ns2_status_get_state() {
    return status.state;
}

void ns2_status_set_last_error(const char *error) {
    snprintf(status.last_error, sizeof(status.last_error), "%s", error ? error : "");
}

void ns2_status_clear_error() {
    status.last_error[0] = 0;
}

void ns2_status_clear_candidates() {
    memset(status.candidates, 0, sizeof(status.candidates));
    status.candidate_seen = 0;
}

void ns2_status_note_candidate(const uint8_t addr[6],
                               uint8_t addr_type,
                               int8_t rssi,
                               const char *name,
                               bool candidate,
                               const char *reason) {
    status.candidate_seen++;
    Ns2Candidate *slot = &status.candidates[(status.candidate_seen - 1) % MAX_CANDIDATES];
    memset(slot, 0, sizeof(*slot));
    slot->used = true;
    slot->candidate = candidate;
    slot->index = status.candidate_seen;
    memcpy(slot->addr, addr, 6);
    slot->addr_type = addr_type;
    slot->rssi = rssi;
    snprintf(slot->name, sizeof(slot->name), "%s", name ? name : "");
    snprintf(slot->reason, sizeof(slot->reason), "%s", reason ? reason : "");
}

uint32_t ns2_status_candidate_count() {
    uint32_t count = 0;
    for (const auto &candidate : status.candidates) {
        if (candidate.used) {
            count++;
        }
    }
    return count;
}

void ns2_status_format_candidates_json(char *out, size_t out_len) {
    if (!out || out_len == 0) {
        return;
    }
    size_t used = static_cast<size_t>(snprintf(out, out_len, "{\"ok\":true,\"profile\":\"ns2pro\",\"candidates\":["));
    bool first = true;
    uint32_t below = UINT32_MAX;

    for (size_t item = 0; item < MAX_CANDIDATES; item++) {
        const Ns2Candidate *best = nullptr;
        for (const auto &candidate : status.candidates) {
            if (!candidate.used || candidate.index >= below) {
                continue;
            }
            if (!best ||
                (candidate.candidate && !best->candidate) ||
                (candidate.candidate == best->candidate && candidate.index > best->index)) {
                best = &candidate;
            }
        }
        if (!best) {
            break;
        }

        below = best->index;
        char addr[32];
        char name[64];
        char reason[64];
        ns2_format_addr(best->addr, best->addr_type, addr, sizeof(addr));
        json_escape(best->name, name, sizeof(name));
        json_escape(best->reason, reason, sizeof(reason));

        const int written = snprintf(out + used,
                                     out_len > used ? out_len - used : 0,
                                     "%s{\"index\":%lu,\"addr\":\"%s\",\"name\":\"%s\",\"rssi\":%d,\"candidate\":%s,\"reason\":\"%s\"}",
                                     first ? "" : ",",
                                     static_cast<unsigned long>(best->index),
                                     addr,
                                     name,
                                     best->rssi,
                                     best->candidate ? "true" : "false",
                                     reason);
        if (written < 0 || static_cast<size_t>(written) >= out_len - used) {
            out[out_len - 1] = 0;
            return;
        }
        used += static_cast<size_t>(written);
        first = false;
    }

    snprintf(out + used, out_len > used ? out_len - used : 0, "]}");
}

void ns2_status_note_connect_attempt() {
    status.connect_attempts++;
}

uint32_t ns2_status_connect_attempts() {
    return status.connect_attempts;
}

void ns2_status_note_connected(const uint8_t addr[6], uint8_t addr_type) {
    memcpy(status.connected_addr, addr, 6);
    status.connected_addr_type = addr_type;
    status.connected_addr_valid = true;
    status.rssi = 0;
}

void ns2_status_note_disconnected(uint8_t reason) {
    status.disconnect_count++;
    status.last_disconnect_reason = reason;
    ns2_status_clear_connection();
}

uint32_t ns2_status_disconnect_count() {
    return status.disconnect_count;
}

uint8_t ns2_status_last_disconnect_reason() {
    return status.last_disconnect_reason;
}

void ns2_status_clear_connection() {
    status.connected_addr_valid = false;
    memset(status.connected_addr, 0, sizeof(status.connected_addr));
    status.connected_addr_type = 0;
    status.rssi = 0;
    status.last_notify_us = 0;
    status.last_notify_len = 0;
}

void ns2_status_set_local_addr(const uint8_t addr[6]) {
    if (!addr) {
        status.local_addr_valid = false;
        memset(status.local_addr, 0, sizeof(status.local_addr));
        return;
    }
    memcpy(status.local_addr, addr, sizeof(status.local_addr));
    status.local_addr_valid = true;
}

void ns2_status_set_rssi(int8_t rssi) {
    status.rssi = rssi;
}

int8_t ns2_status_rssi() {
    return status.rssi;
}

void ns2_status_note_notify(uint16_t len) {
    status.notify_count++;
    status.last_notify_len = len;
    status.last_notify_us = time_us_64();
    update_notify_hz(status.last_notify_us);
}

uint32_t ns2_status_notify_count() {
    return status.notify_count;
}

uint32_t ns2_status_notify_hz() {
    update_notify_hz(time_us_64());
    return status.notify_hz;
}

uint32_t ns2_status_last_notify_age_ms() {
    if (status.last_notify_us == 0) {
        return UINT32_MAX;
    }
    return static_cast<uint32_t>((time_us_64() - status.last_notify_us) / 1000ULL);
}

bool ns2_status_live_notify() {
    return status.last_notify_us != 0 &&
           (time_us_64() - status.last_notify_us) <= LIVE_NOTIFY_STALE_US;
}

const char *ns2_status_last_error() {
    return status.last_error;
}

void ns2_status_format_json(char *out, size_t out_len) {
    if (!out || out_len == 0) {
        return;
    }

    update_notify_hz(time_us_64());

    char saved[32] = "";
    char local[32] = "";
    char connected[32] = "";
    char error[96];
    uint8_t saved_addr[6];
    uint8_t saved_type = 0;
    if (ns2_config_has_saved_target()) {
        ns2_config_get_saved_target(saved_addr, &saved_type);
        ns2_format_addr(saved_addr, saved_type, saved, sizeof(saved));
    }
    if (status.local_addr_valid) {
        ns2_format_addr(status.local_addr, 0, local, sizeof(local));
    }
    if (status.connected_addr_valid) {
        ns2_format_addr(status.connected_addr, status.connected_addr_type, connected, sizeof(connected));
    }
    json_escape(status.last_error, error, sizeof(error));

    const uint32_t age_ms = ns2_status_last_notify_age_ms();
    snprintf(out, out_len,
             "{\"ok\":true,\"profile\":\"ns2pro\",\"ble_state\":\"%s\",\"auto_connect\":\"%s\","
             "\"local_addr\":\"%s\",\"saved_target\":\"%s\",\"candidate_count\":%lu,\"connected_addr\":\"%s\","
             "\"rssi\":%d,\"notify_count\":%lu,\"notify_hz\":%lu,\"last_notify_age_ms\":%ld,"
             "\"connect_attempts\":%lu,\"disconnect_count\":%lu,\"bond_count\":%d,\"last_error\":\"%s\"}",
             ns2_ble_state_name(status.state),
             ns2_config_auto_connect_enabled() ? "on" : "off",
             local,
             saved,
             static_cast<unsigned long>(ns2_status_candidate_count()),
             connected,
             status.rssi,
             static_cast<unsigned long>(status.notify_count),
             static_cast<unsigned long>(status.notify_hz),
             age_ms == UINT32_MAX ? -1L : static_cast<long>(age_ms),
             static_cast<unsigned long>(status.connect_attempts),
             static_cast<unsigned long>(status.disconnect_count),
             le_device_db_count(),
             error);
}

void ns2_status_tick_led() {
    const uint64_t now = time_us_64();
    const uint64_t elapsed = now - led_epoch_us;
    bool on = false;

    switch (status.state) {
        case Ns2BleState::Scanning:
            on = (elapsed % 1'000'000ULL) < 500'000ULL;
            break;
        case Ns2BleState::Connecting:
        case Ns2BleState::Pairing:
        case Ns2BleState::Discovering:
        case Ns2BleState::Subscribing:
        case Ns2BleState::InitializingController:
            on = (elapsed % 250'000ULL) < 125'000ULL;
            break;
        case Ns2BleState::ConnectedLive:
            on = true;
            break;
        case Ns2BleState::ConnectedNoNotify:
            on = pulse_train(elapsed, 2);
            break;
        case Ns2BleState::Backoff:
        case Ns2BleState::Error:
            on = pulse_train(elapsed, 3);
            break;
        case Ns2BleState::Boot:
        case Ns2BleState::BleInit:
        case Ns2BleState::Idle:
        case Ns2BleState::Disconnected:
        default:
            on = false;
            break;
    }

    set_led(on);
}
