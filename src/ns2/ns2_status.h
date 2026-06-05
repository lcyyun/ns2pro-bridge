#ifndef DS5_BRIDGE_NS2_STATUS_H
#define DS5_BRIDGE_NS2_STATUS_H

#include <cstddef>
#include <cstdint>

#include "ns2_state.h"

struct Ns2Candidate {
    bool used;
    bool candidate;
    uint32_t index;
    uint8_t addr[6];
    uint8_t addr_type;
    int8_t rssi;
    char name[32];
    char reason[32];
};

void ns2_status_init();
void ns2_status_set_state(Ns2BleState state);
Ns2BleState ns2_status_get_state();

void ns2_status_set_last_error(const char *error);
void ns2_status_clear_error();

void ns2_status_clear_candidates();
void ns2_status_note_candidate(const uint8_t addr[6],
                               uint8_t addr_type,
                               int8_t rssi,
                               const char *name,
                               bool candidate,
                               const char *reason);
uint32_t ns2_status_candidate_count();
void ns2_status_format_candidates_json(char *out, size_t out_len);

void ns2_status_note_connect_attempt();
uint32_t ns2_status_connect_attempts();
void ns2_status_note_connected(const uint8_t addr[6], uint8_t addr_type);
void ns2_status_note_disconnected(uint8_t reason);
uint32_t ns2_status_disconnect_count();
uint8_t ns2_status_last_disconnect_reason();
void ns2_status_clear_connection();
void ns2_status_set_local_addr(const uint8_t addr[6]);
void ns2_status_set_rssi(int8_t rssi);
int8_t ns2_status_rssi();

void ns2_status_note_notify(uint16_t len);
uint32_t ns2_status_notify_count();
uint32_t ns2_status_notify_hz();
uint32_t ns2_status_last_notify_age_ms();
bool ns2_status_live_notify();
const char *ns2_status_last_error();

void ns2_status_format_json(char *out, size_t out_len);
void ns2_status_tick_led();

#endif
