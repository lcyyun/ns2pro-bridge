#pragma once

#include "esp_err.h"

namespace ns2 {

struct BleStats {
    bool started = false;
    bool scanning = false;
    bool connected = false;
    bool gatt_ready = false;
    bool input_subscribed = false;
    bool rumble_ready = false;
    bool local_addr_valid = false;
    bool auto_connect = true;
    bool saved_target_valid = false;
    bool pair_mode = false;
    uint8_t local_addr[6] = {};
    uint8_t saved_addr[6] = {};
    uint8_t saved_addr_type = 0;
    uint8_t own_addr_type = 0;
    uint32_t candidates = 0;
    uint32_t scan_starts = 0;
    uint32_t scan_errors = 0;
    uint32_t adv_seen = 0;
    uint32_t adv_logged = 0;
    uint32_t connect_attempts = 0;
    uint32_t disconnect_count = 0;
    uint32_t services = 0;
    uint32_t characteristics = 0;
    uint32_t notify_count = 0;
    uint32_t notify_hz = 0;
    uint32_t notify_last_gap_us = 0;
    uint32_t notify_max_gap_us = 0;
    uint16_t conn_interval_units = 0;
    uint32_t conn_interval_us = 0;
    uint16_t conn_latency = 0;
    uint16_t conn_supervision_timeout = 0;
    uint32_t conn_update_requests = 0;
    int conn_update_start_rc = 0;
    int conn_update_status = 0;
    uint8_t tx_phy = 0;
    uint8_t rx_phy = 0;
    int phy_update_rc = 0;
    int phy_update_status = 0;
    int data_len_update_rc = 0;
    uint16_t data_len_tx_octets = 0;
    uint16_t data_len_rx_octets = 0;
    uint16_t data_len_tx_time = 0;
    uint16_t data_len_rx_time = 0;
    uint32_t rumble_writes = 0;
    uint32_t rumble_errors = 0;
    int last_error = 0;
    uint8_t last_disconnect_reason = 0;
    int8_t last_rssi = 0;
    char last_addr[18] = {};
    char last_name[32] = {};
};

esp_err_t ble_start();
void ble_task();
void ble_get_stats(BleStats *out);
bool ble_send_rumble(const uint8_t *data, uint16_t len);
void ble_start_scan();
void ble_pair();
void ble_reconnect();
void ble_disconnect();
void ble_forget();
void ble_set_auto_connect(bool enabled);

} // namespace ns2
