#ifndef DS5_BRIDGE_NS2_BLE_H
#define DS5_BRIDGE_NS2_BLE_H

#include <cstdint>

void ns2_ble_init();
void ns2_ble_tick();

void ns2_ble_start_scan();
void ns2_ble_pair();
void ns2_ble_reconnect();
void ns2_ble_disconnect();
void ns2_ble_forget();
void ns2_ble_set_auto_connect(bool enabled);
bool ns2_ble_auto_connect_enabled();
bool ns2_ble_rumble_ready();
bool ns2_ble_send_rumble(const uint8_t *data, uint16_t len);

#endif
