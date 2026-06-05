#include "ns2_ble.h"

#include <cctype>
#include <cstdio>
#include <cstring>

#include "ble/le_device_db.h"
#include "btstack.h"
#include "ns2_gatt.h"
#include "ns2_input.h"
#include "ns2_state.h"
#include "ns2_status.h"
#include "pico/time.h"

#ifndef ATT_PROPERTY_WRITE_WITHOUT_RESPONSE
#define ATT_PROPERTY_WRITE_WITHOUT_RESPONSE 0x04
#endif

namespace {

constexpr uint16_t SCAN_INTERVAL = 0x0010;
constexpr uint16_t SCAN_WINDOW = 0x0010;
constexpr uint16_t FAST_CONN_INTERVAL_MIN = 0x0006;
constexpr uint16_t FAST_CONN_INTERVAL_MAX = 0x0006;
constexpr uint16_t FAST_CONN_LATENCY = 0;
constexpr uint16_t FAST_CONN_SUPERVISION_TIMEOUT = 100;
constexpr uint16_t NS2_LE_MTU = 256;
constexpr int8_t STRONG_CANDIDATE_RSSI = -75;
constexpr uint32_t AUTO_START_DELAY_MS = 600;
constexpr uint32_t CONNECT_WATCHDOG_MS = 30000;
constexpr uint32_t GATT_DISCOVERY_DELAY_MS = 0;
constexpr uint32_t SAVED_RECONNECT_GATT_DELAY_MS = 300;
constexpr uint32_t GATT_DISCOVERY_RETRY_MS = 50;
constexpr uint8_t MAX_GATT_DISCOVERY_RETRIES = 20;
constexpr uint32_t MTU_EXCHANGE_TIMEOUT_MS = 1000;
constexpr uint32_t SECURITY_WAIT_MS = 3500;
constexpr uint32_t INIT_ACK_TIMEOUT_MS = 3500;
constexpr uint32_t NOTIFY_TIMEOUT_MS = 5000;
constexpr uint32_t RSSI_POLL_MS = 5000;
constexpr uint16_t NINTENDO_COMPANY_ID = 0x0553;
constexpr uint8_t NS2_MANUFACTURER_PREFIX[] = {0x01, 0x00, 0x03, 0x7e};
constexpr size_t MAX_ADV_MFG_DATA = 31;
constexpr size_t MAX_SERVICES = 24;
constexpr size_t MAX_CHARS = 72;
constexpr uint8_t ADV_UUID_NOTIFY_FD2[16] = {
    0xab, 0x7d, 0xe9, 0xbe, 0x89, 0xfe, 0x49, 0xad,
    0x82, 0x8f, 0x11, 0x8f, 0x09, 0xdf, 0x7f, 0xd2
};
constexpr uint8_t UUID_NOTIFY_LEGACY[16] = {
    0x74, 0x92, 0x86, 0x6c, 0xec, 0x3e, 0x46, 0x19,
    0x82, 0x58, 0x32, 0x75, 0x5f, 0xfc, 0xc0, 0xf8
};
constexpr uint8_t UUID_ACK[16] = {
    0xc7, 0x65, 0xa9, 0x61, 0xd9, 0xd8, 0x4d, 0x36,
    0xa2, 0x0a, 0x53, 0x15, 0xb1, 0x11, 0x83, 0x6a
};
constexpr uint8_t UUID_COMMAND[16] = {
    0x64, 0x9d, 0x4a, 0xc9, 0x8e, 0xb7, 0x4e, 0x6c,
    0xaf, 0x44, 0x1e, 0xa5, 0x4f, 0xe5, 0xf0, 0x05
};

constexpr uint16_t KNOWN_INPUT_FD2_VALUE_HANDLE = 0x000a;
constexpr uint16_t KNOWN_COMMAND_VALUE_HANDLE = 0x0014;
constexpr uint16_t KNOWN_ACK_VALUE_HANDLE = 0x001a;
constexpr uint16_t NS2_ATT_PROPERTY_NOTIFY = 0x10;

enum class ConnectMode : uint8_t {
    None = 0,
    DirectSaved,
    ScanAuto,
    ManualScan,
    PairingScan
};

struct AdvertisementInfo {
    char name[32];
    bool nintendo_mfg;
    bool ns2_mfg_prefix;
    bool ns2_pairing_mfg;
    bool service_match;
    bool appearance_match;
    uint8_t nintendo_data[MAX_ADV_MFG_DATA];
    uint8_t nintendo_data_len;
};

enum class GattStage : uint8_t {
    Idle = 0,
    MtuExchange,
    DiscoverServices,
    DiscoverCharacteristics,
    SubscribeAck,
    Initializing,
    SubscribeInput,
    Ready
};

struct ServiceSlot {
    bool used;
    gatt_client_service_t service;
};

struct CharacteristicSlot {
    bool used;
    gatt_client_characteristic_t characteristic;
    gatt_client_notification_t notification;
    Ns2GattRole role;
    bool subscribed;
};

struct BleRuntime {
    bool hci_ready;
    bool scanning;
    bool connecting;
    bool direct_saved_tried;
    bool pending_connect_valid;
    bool target_saved_this_connection;
    bool current_addr_valid;
    ConnectMode connect_mode;
    hci_con_handle_t con_handle;
    uint8_t pending_addr[6];
    uint8_t pending_addr_type;
    uint8_t current_addr[6];
    uint8_t current_addr_type;
    uint32_t adv_log_count;
    uint64_t auto_start_due_ms;
    uint64_t connect_started_ms;
    uint64_t security_started_ms;
    uint64_t mtu_exchange_due_ms;
    uint64_t mtu_exchange_started_ms;
    uint64_t service_discovery_due_ms;
    uint64_t known_handle_due_ms;
    uint64_t retry_window_started_ms;
    uint64_t next_retry_ms;
    uint64_t last_rssi_poll_ms;

    ServiceSlot services[MAX_SERVICES];
    CharacteristicSlot chars[MAX_CHARS];
    size_t service_count;
    size_t char_count;
    size_t discover_service_index;
    int subscribe_index;
    GattStage gatt_stage;
    uint16_t command_value_handle;
    uint16_t rumble_value_handle;
    bool command_write_without_response;
    bool rumble_write_without_response;
    bool security_pending;
    bool security_required;
    bool mtu_exchange_pending;
    bool pair_required_after_disconnect;
    uint8_t init_index;
    uint8_t service_discovery_retries;
    uint64_t init_command_sent_ms;
    uint8_t cccd_enable_notify[2];
};

BleRuntime ble{};
btstack_packet_callback_registration_t hci_event_callback_registration{};
btstack_packet_callback_registration_t sm_event_callback_registration{};

void hci_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
void gatt_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
void sm_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);

uint64_t now_ms() {
    return time_us_64() / 1000ULL;
}

void clear_gatt_results() {
    memset(ble.services, 0, sizeof(ble.services));
    memset(ble.chars, 0, sizeof(ble.chars));
    ble.service_count = 0;
    ble.char_count = 0;
    ble.discover_service_index = 0;
    ble.subscribe_index = -1;
    ble.command_value_handle = 0;
    ble.rumble_value_handle = 0;
    ble.command_write_without_response = false;
    ble.rumble_write_without_response = false;
    ble.security_pending = false;
    ble.security_required = false;
    ble.pair_required_after_disconnect = false;
    ble.security_started_ms = 0;
    ble.mtu_exchange_pending = false;
    ble.mtu_exchange_started_ms = 0;
    ble.mtu_exchange_due_ms = 0;
    ble.known_handle_due_ms = 0;
    ble.init_index = 0;
    ble.init_command_sent_ms = 0;
}

void reset_gatt_cache() {
    clear_gatt_results();
    ble.gatt_stage = GattStage::Idle;
    ble.service_discovery_retries = 0;
    ble.service_discovery_due_ms = 0;
}

void print_le_db_summary() {
    const int count = le_device_db_count();
    const int max_count = le_device_db_max_count();
    printf("[NS2 SM] LE device DB count=%d max=%d\n", count, max_count);
    for (int i = 0; i < max_count; i++) {
        int addr_type = 0;
        bd_addr_t addr{};
        sm_key_t irk{};
        le_device_db_info(i, &addr_type, addr, irk);
        if (ns2_addr_is_empty(addr)) {
            continue;
        }
        printf("[NS2 SM] LE DB[%d] type=%d addr=%s irk0=%02x%02x\n",
               i,
               addr_type,
               bd_addr_to_str(addr),
               irk[0],
               irk[1]);
    }
}

void configure_fast_connection_parameters() {
    gap_set_connection_parameters(SCAN_INTERVAL,
                                  SCAN_WINDOW,
                                  FAST_CONN_INTERVAL_MIN,
                                  FAST_CONN_INTERVAL_MAX,
                                  FAST_CONN_LATENCY,
                                  FAST_CONN_SUPERVISION_TIMEOUT,
                                  0,
                                  0);
}

void request_fast_connection_update(const char *reason) {
    if (ble.con_handle == HCI_CON_HANDLE_INVALID) {
        return;
    }

    const int status = gap_update_connection_parameters(ble.con_handle,
                                                        FAST_CONN_INTERVAL_MIN,
                                                        FAST_CONN_INTERVAL_MAX,
                                                        FAST_CONN_LATENCY,
                                                        FAST_CONN_SUPERVISION_TIMEOUT);
    printf("[NS2 HCI] fast conn update reason=%s interval=%u..%u latency=%u supervision=%u status=%d\n",
           reason ? reason : "unknown",
           FAST_CONN_INTERVAL_MIN,
           FAST_CONN_INTERVAL_MAX,
           FAST_CONN_LATENCY,
           FAST_CONN_SUPERVISION_TIMEOUT,
           status);
}

void save_current_target_if_live(const char *reason) {
    if (!ble.current_addr_valid || ble.target_saved_this_connection) {
        return;
    }

    uint8_t saved_addr[6];
    uint8_t saved_type = 0;
    const bool has_saved = ns2_config_has_saved_target();
    if (has_saved) {
        ns2_config_get_saved_target(saved_addr, &saved_type);
        if (ns2_addr_equal(saved_addr, ble.current_addr) && saved_type == ble.current_addr_type) {
            ble.target_saved_this_connection = true;
            return;
        }
    }

    char formatted[32];
    ns2_format_addr(ble.current_addr, ble.current_addr_type, formatted, sizeof(formatted));
    ns2_config_set_saved_target(ble.current_addr,
                                ble.current_addr_type,
                                static_cast<uint32_t>(now_ms()));
    const bool ok = ns2_config_save();
    ble.target_saved_this_connection = ok;
    printf("[NS2 BLE] saved target after %s addr=%s ok=%u\n",
           reason ? reason : "live",
           formatted,
           ok ? 1u : 0u);
}

bool contains_ci(const char *haystack, const char *needle) {
    if (!haystack || !needle || !needle[0]) {
        return false;
    }
    const size_t needle_len = strlen(needle);
    for (const char *p = haystack; *p; p++) {
        size_t i = 0;
        while (i < needle_len && p[i] &&
               static_cast<char>(tolower(static_cast<unsigned char>(p[i]))) ==
                   static_cast<char>(tolower(static_cast<unsigned char>(needle[i])))) {
            i++;
        }
        if (i == needle_len) {
            return true;
        }
    }
    return false;
}

bool name_looks_like_ns2(const char *name) {
    return contains_ci(name, "switch") ||
           contains_ci(name, "nintendo") ||
           contains_ci(name, "pro controller") ||
           contains_ci(name, "pro2") ||
           contains_ci(name, "ns2");
}

bool uuid128_adv_matches(const uint8_t *adv_uuid_le, const uint8_t uuid_be[16]) {
    for (size_t i = 0; i < 16; i++) {
        if (adv_uuid_le[i] != uuid_be[15 - i]) {
            return false;
        }
    }
    return true;
}

bool data_starts_with(const uint8_t *data, uint8_t data_len, const uint8_t *prefix, size_t prefix_len) {
    return data && prefix && data_len >= prefix_len && memcmp(data, prefix, prefix_len) == 0;
}

bool ns2_mfg_looks_pairable(const uint8_t *data, uint8_t data_len) {
    if (!data || data_len < 16) {
        return false;
    }
    if (!data_starts_with(data, data_len, NS2_MANUFACTURER_PREFIX, sizeof(NS2_MANUFACTURER_PREFIX))) {
        return false;
    }

    for (uint8_t i = 10; i < 16; i++) {
        if (data[i] != 0) {
            return false;
        }
    }
    return true;
}

void format_hex(const uint8_t *data, uint8_t len, char *out, size_t out_len) {
    if (!out || out_len == 0) {
        return;
    }
    size_t used = 0;
    for (uint8_t i = 0; i < len && used + 2 < out_len; i++) {
        const int written = snprintf(out + used,
                                     out_len - used,
                                     "%s%02x",
                                     i == 0 ? "" : " ",
                                     data[i]);
        if (written < 0 || static_cast<size_t>(written) >= out_len - used) {
            out[out_len - 1] = 0;
            return;
        }
        used += static_cast<size_t>(written);
    }
    out[used] = 0;
}

void parse_advertisement(const uint8_t *data,
                         uint8_t data_len,
                         AdvertisementInfo *info) {
    if (!info) {
        return;
    }
    memset(info, 0, sizeof(*info));

    uint8_t offset = 0;
    while (offset + 1 < data_len) {
        const uint8_t field_len = data[offset];
        if (field_len == 0 || offset + field_len >= data_len + 1) {
            break;
        }
        const uint8_t type = data[offset + 1];
        const uint8_t *payload = &data[offset + 2];
        const uint8_t payload_len = field_len - 1;

        switch (type) {
            case 0x08:
            case 0x09: {
                if (payload_len > 0) {
                    const size_t copy_len = payload_len < sizeof(info->name) - 1 ? payload_len : sizeof(info->name) - 1;
                    memcpy(info->name, payload, copy_len);
                    info->name[copy_len] = 0;
                }
                break;
            }
            case 0xff: {
                if (payload_len >= 2) {
                    const uint16_t company = static_cast<uint16_t>(payload[0]) |
                                             (static_cast<uint16_t>(payload[1]) << 8);
                    if (company == NINTENDO_COMPANY_ID) {
                        info->nintendo_mfg = true;
                        const uint8_t value_len = static_cast<uint8_t>(payload_len - 2);
                        info->nintendo_data_len = value_len < MAX_ADV_MFG_DATA ? value_len : MAX_ADV_MFG_DATA;
                        memcpy(info->nintendo_data, payload + 2, info->nintendo_data_len);
                        info->ns2_mfg_prefix = data_starts_with(info->nintendo_data,
                                                                info->nintendo_data_len,
                                                                NS2_MANUFACTURER_PREFIX,
                                                                sizeof(NS2_MANUFACTURER_PREFIX));
                        info->ns2_pairing_mfg = ns2_mfg_looks_pairable(info->nintendo_data,
                                                                       info->nintendo_data_len);
                    }
                }
                break;
            }
            case 0x19: {
                if (payload_len >= 2) {
                    const uint16_t appearance = static_cast<uint16_t>(payload[0]) |
                                                (static_cast<uint16_t>(payload[1]) << 8);
                    if ((appearance & 0xffc0) == 0x03c0) {
                        info->appearance_match = true;
                    }
                }
                break;
            }
            case 0x06:
            case 0x07: {
                for (uint8_t i = 0; i + 16 <= payload_len; i += 16) {
                    if (uuid128_adv_matches(payload + i, ADV_UUID_NOTIFY_FD2)) {
                        info->service_match = true;
                    }
                }
                break;
            }
            default:
                break;
        }

        offset = static_cast<uint8_t>(offset + field_len + 1);
    }
}

uint32_t retry_delay_ms() {
    const uint64_t elapsed = ble.retry_window_started_ms == 0 ? 0 : now_ms() - ble.retry_window_started_ms;
    if (elapsed < 30'000ULL) {
        return 2000;
    }
    if (elapsed < 120'000ULL) {
        return 5000;
    }
    return 10000;
}

void enter_backoff(const char *error) {
    if (error && error[0]) {
        ns2_status_set_last_error(error);
    }
    if (ble.retry_window_started_ms == 0) {
        ble.retry_window_started_ms = now_ms();
    }
    ble.next_retry_ms = now_ms() + retry_delay_ms();
    ble.scanning = false;
    ble.connecting = false;
    ble.direct_saved_tried = false;
    ble.pending_connect_valid = false;
    ble.current_addr_valid = false;
    ble.connect_mode = ConnectMode::None;
    reset_gatt_cache();
    ns2_status_set_state(Ns2BleState::Backoff);
}

void reset_retry_window() {
    ble.retry_window_started_ms = 0;
    ble.next_retry_ms = 0;
}

bool start_connect(const uint8_t addr[6], uint8_t addr_type, ConnectMode mode, const char *label) {
    if (!ble.hci_ready) {
        ns2_status_set_last_error("BLE stack is not ready");
        return false;
    }

    if (ble.scanning) {
        gap_stop_scan();
        ble.scanning = false;
    }

    ns2_status_note_connect_attempt();
    memcpy(ble.pending_addr, addr, 6);
    ble.pending_addr_type = addr_type;
    ble.pending_connect_valid = true;
    ble.current_addr_valid = false;
    ble.target_saved_this_connection = false;
    ble.connect_mode = mode;
    ble.connecting = true;
    ble.connect_started_ms = now_ms();
    reset_gatt_cache();
    ns2_status_set_state(Ns2BleState::Connecting);

    char formatted[32];
    ns2_format_addr(addr, addr_type, formatted, sizeof(formatted));
    printf("[NS2 BLE] connect start %s target=%s\n", label ? label : "", formatted);

    configure_fast_connection_parameters();
    const uint8_t status = gap_connect(const_cast<uint8_t *>(addr), static_cast<bd_addr_type_t>(addr_type));
    if (status != ERROR_CODE_SUCCESS) {
        printf("[NS2 BLE] gap_connect failed status=0x%02x\n", status);
        ble.connecting = false;
        ble.pending_connect_valid = false;
        char error[32];
        snprintf(error, sizeof(error), "gap fail 0x%02x", status);
        enter_backoff(error);
        return false;
    }
    return true;
}

bool start_scan_internal(ConnectMode mode) {
    if (!ble.hci_ready) {
        ns2_status_set_last_error("BLE stack is not ready");
        return false;
    }
    if (ble.connecting || ble.con_handle != HCI_CON_HANDLE_INVALID) {
        return false;
    }

    if (ble.scanning) {
        gap_stop_scan();
    }

    ns2_status_clear_candidates();
    ble.adv_log_count = 0;
    gap_set_scan_parameters(1, SCAN_INTERVAL, SCAN_WINDOW);
    gap_start_scan();
    ble.scanning = true;
    ble.connect_mode = mode;
    ns2_status_set_state(Ns2BleState::Scanning);
    printf("[NS2 BLE] active scan started mode=%u\n", static_cast<unsigned>(mode));
    return true;
}

void start_auto_attempt() {
    if (!ns2_config_auto_connect_enabled()) {
        ns2_status_set_state(Ns2BleState::Idle);
        return;
    }

    ns2_status_clear_error();
    printf("[NS2 BLE] auto pair scan started; saved-address reconnect disabled bond_count=%d\n",
           le_device_db_count());
    start_scan_internal(ConnectMode::PairingScan);
}

void clear_saved_target_after_security_failure() {
    if (ble.connect_mode == ConnectMode::PairingScan || !ns2_config_has_saved_target()) {
        return;
    }

    ns2_config_forget_saved_target();
    ns2_config_set_auto_connect(false);
    const bool ok = ns2_config_save();
    ble.pair_required_after_disconnect = true;
    ns2_status_set_last_error("pair required");
    printf("[NS2 BLE] saved target security failed; cleared saved target and disabled auto ok=%u; run ns2 pair\n",
           ok ? 1u : 0u);
}

void disconnect_and_backoff(const char *error) {
    if (ble.con_handle != HCI_CON_HANDLE_INVALID) {
        gap_disconnect(ble.con_handle);
        if (error && error[0]) {
            ns2_status_set_last_error(error);
        }
        return;
    }
    enter_backoff(error);
}

CharacteristicSlot *find_char_by_role(Ns2GattRole role) {
    for (size_t i = 0; i < ble.char_count; i++) {
        if (ble.chars[i].used && ble.chars[i].role == role) {
            return &ble.chars[i];
        }
    }
    return nullptr;
}

CharacteristicSlot *find_char_by_value_handle(uint16_t value_handle) {
    for (size_t i = 0; i < ble.char_count; i++) {
        if (ble.chars[i].used && ble.chars[i].characteristic.value_handle == value_handle) {
            return &ble.chars[i];
        }
    }
    return nullptr;
}

bool subscribe_next(Ns2GattRole role);

void add_known_characteristic(Ns2GattRole role,
                              uint16_t value_handle,
                              uint16_t properties,
                              const uint8_t uuid128[16]) {
    if (ble.char_count >= MAX_CHARS) {
        return;
    }

    CharacteristicSlot &slot = ble.chars[ble.char_count++];
    memset(&slot, 0, sizeof(slot));
    slot.used = true;
    slot.role = role;
    slot.characteristic.start_handle = static_cast<uint16_t>(value_handle > 0 ? value_handle - 1 : 0);
    slot.characteristic.value_handle = value_handle;
    slot.characteristic.end_handle = static_cast<uint16_t>(value_handle + 1);
    slot.characteristic.properties = properties;
    slot.characteristic.uuid16 = 0;
    memcpy(slot.characteristic.uuid128, uuid128, NS2_UUID128_LEN);
}

void start_known_handle_flow() {
    clear_gatt_results();
    add_known_characteristic(Ns2GattRole::AckNotify,
                             KNOWN_ACK_VALUE_HANDLE,
                             NS2_ATT_PROPERTY_NOTIFY,
                             UUID_ACK);
    add_known_characteristic(Ns2GattRole::Command,
                             KNOWN_COMMAND_VALUE_HANDLE,
                             ATT_PROPERTY_WRITE_WITHOUT_RESPONSE,
                             UUID_COMMAND);
    add_known_characteristic(Ns2GattRole::InputNotify,
                             KNOWN_INPUT_FD2_VALUE_HANDLE,
                             static_cast<uint16_t>(NS2_ATT_PROPERTY_NOTIFY | 0x02),
                             ADV_UUID_NOTIFY_FD2);

    ble.gatt_stage = GattStage::SubscribeAck;
    ble.subscribe_index = -1;
    ns2_status_set_state(Ns2BleState::Subscribing);
    printf("[NS2 GATT] using known NS2 handles for saved reconnect chars=%lu\n",
           static_cast<unsigned long>(ble.char_count));

    if (!subscribe_next(Ns2GattRole::AckNotify)) {
        disconnect_and_backoff("known ACK subscribe failed");
    }
}

void schedule_mtu_exchange(uint32_t delay_ms) {
    clear_gatt_results();
    ble.gatt_stage = GattStage::MtuExchange;
    ble.mtu_exchange_due_ms = now_ms() + delay_ms;
    ns2_status_set_state(Ns2BleState::Discovering);
    printf("[NS2 GATT] MTU exchange scheduled delay=%lu\n",
           static_cast<unsigned long>(delay_ms));
}

void start_mtu_exchange() {
    clear_gatt_results();
    ble.gatt_stage = GattStage::MtuExchange;
    ble.mtu_exchange_pending = true;
    ble.mtu_exchange_started_ms = now_ms();
    ns2_status_set_state(Ns2BleState::Discovering);
    printf("[NS2 GATT] MTU exchange start\n");
    gatt_client_send_mtu_negotiation(gatt_packet_handler, ble.con_handle);
}

void schedule_service_discovery(uint32_t delay_ms) {
    clear_gatt_results();
    ble.gatt_stage = GattStage::DiscoverServices;
    ble.service_discovery_due_ms = now_ms() + delay_ms;
    ns2_status_set_state(Ns2BleState::Discovering);
    printf("[NS2 GATT] service discovery scheduled delay=%lu retry=%u\n",
           static_cast<unsigned long>(delay_ms),
           static_cast<unsigned>(ble.service_discovery_retries));
}

void start_service_discovery() {
    clear_gatt_results();
    ble.gatt_stage = GattStage::DiscoverServices;
    ble.service_discovery_due_ms = 0;
    ns2_status_set_state(Ns2BleState::Discovering);
    const uint8_t status = gatt_client_discover_primary_services(gatt_packet_handler, ble.con_handle);
    if (status != ERROR_CODE_SUCCESS) {
        printf("[NS2 GATT] service discovery start failed status=0x%02x\n", status);
        if (ble.service_discovery_retries < MAX_GATT_DISCOVERY_RETRIES) {
            ble.service_discovery_retries++;
            schedule_service_discovery(GATT_DISCOVERY_RETRY_MS);
        } else {
            disconnect_and_backoff("service discovery failed");
        }
    } else {
        printf("[NS2 GATT] service discovery start retry=%u\n",
               static_cast<unsigned>(ble.service_discovery_retries));
    }
}

void discover_next_service_characteristics() {
    while (ble.discover_service_index < ble.service_count) {
        ServiceSlot &slot = ble.services[ble.discover_service_index];
        if (!slot.used) {
            ble.discover_service_index++;
            continue;
        }
        ble.gatt_stage = GattStage::DiscoverCharacteristics;
        const uint8_t status = gatt_client_discover_characteristics_for_service(
            gatt_packet_handler,
            ble.con_handle,
            &slot.service);
        if (status == ERROR_CODE_SUCCESS) {
            printf("[NS2 GATT] characteristic discovery svc=%lu\n",
                   static_cast<unsigned long>(ble.discover_service_index));
            return;
        }
        printf("[NS2 GATT] characteristic discovery start failed svc=%lu status=0x%02x\n",
               static_cast<unsigned long>(ble.discover_service_index),
               status);
        ble.discover_service_index++;
    }

    printf("[NS2 GATT] discovery complete services=%lu chars=%lu\n",
           static_cast<unsigned long>(ble.service_count),
           static_cast<unsigned long>(ble.char_count));
    ble.subscribe_index = -1;
    ble.gatt_stage = GattStage::SubscribeAck;
    ns2_status_set_state(Ns2BleState::Subscribing);
    if (!subscribe_next(Ns2GattRole::AckNotify)) {
        disconnect_and_backoff("ACK notify characteristic missing");
    }
}

bool subscribe_next(Ns2GattRole role) {
    for (int i = ble.subscribe_index + 1; i < static_cast<int>(ble.char_count); i++) {
        CharacteristicSlot &slot = ble.chars[i];
        if (!slot.used || slot.role != role || slot.subscribed) {
            continue;
        }

        ble.subscribe_index = i;
        gatt_client_listen_for_characteristic_value_updates(
            &slot.notification,
            gatt_packet_handler,
            ble.con_handle,
            &slot.characteristic);

        ble.cccd_enable_notify[0] = 0x01;
        ble.cccd_enable_notify[1] = 0x00;
        const uint16_t cccd_handle = static_cast<uint16_t>(slot.characteristic.value_handle + 1);
        const uint8_t status = gatt_client_write_value_of_characteristic(
            gatt_packet_handler,
            ble.con_handle,
            cccd_handle,
            sizeof(ble.cccd_enable_notify),
            ble.cccd_enable_notify);
        if (status == ERROR_CODE_SUCCESS) {
            char uuid[48];
            ns2_gatt_format_uuid(slot.characteristic.uuid16, slot.characteristic.uuid128, uuid, sizeof(uuid));
            printf("[NS2 GATT] subscribe start role=%s uuid=%s cccd=0x%04x\n",
                   ns2_gatt_role_name(role),
                   uuid,
                   cccd_handle);
            return true;
        }

        printf("[NS2 GATT] subscribe start failed role=%s status=0x%02x\n",
               ns2_gatt_role_name(role),
               status);
    }
    return false;
}

void start_input_subscriptions() {
    ble.gatt_stage = GattStage::SubscribeInput;
    ble.subscribe_index = -1;
    ns2_status_set_state(Ns2BleState::Subscribing);
    if (!subscribe_next(Ns2GattRole::InputNotify)) {
        disconnect_and_backoff("input notify characteristic missing");
    }
}

bool write_command(const uint8_t *data, uint16_t len) {
    if (ble.command_value_handle == 0) {
        return false;
    }
    uint8_t status;
    if (ble.command_write_without_response) {
        status = gatt_client_write_value_of_characteristic_without_response(
            ble.con_handle,
            ble.command_value_handle,
            len,
            const_cast<uint8_t *>(data));
    } else {
        status = gatt_client_write_value_of_characteristic(
            gatt_packet_handler,
            ble.con_handle,
            ble.command_value_handle,
            len,
            const_cast<uint8_t *>(data));
    }
    return status == ERROR_CODE_SUCCESS;
}

void send_current_init_command() {
    if (ble.init_index >= ns2_gatt_init_command_count()) {
        printf("[NS2 GATT] controller init complete; enabling input notify\n");
        start_input_subscriptions();
        return;
    }

    const Ns2InitCommand &cmd = ns2_gatt_init_command(ble.init_index);
    if (!write_command(cmd.data, cmd.len)) {
        disconnect_and_backoff("controller init write failed");
        return;
    }
    ble.init_command_sent_ms = now_ms();
    ns2_status_set_state(Ns2BleState::InitializingController);
    printf("[NS2 GATT] init send %u/%u %s len=%u\n",
           static_cast<unsigned>(ble.init_index + 1),
           static_cast<unsigned>(ns2_gatt_init_command_count()),
           cmd.name,
           static_cast<unsigned>(cmd.len));
}

void start_controller_init() {
    CharacteristicSlot *cmd = find_char_by_role(Ns2GattRole::Command);
    if (!cmd) {
        disconnect_and_backoff("command characteristic missing");
        return;
    }
    ble.command_value_handle = cmd->characteristic.value_handle;
    ble.command_write_without_response =
        (cmd->characteristic.properties & ATT_PROPERTY_WRITE_WITHOUT_RESPONSE) != 0;

    ble.gatt_stage = GattStage::Initializing;
    ble.init_index = 0;
    send_current_init_command();
}

void handle_gatt_query_complete(uint8_t att_status) {
    if (att_status != ATT_ERROR_SUCCESS) {
        printf("[NS2 GATT] query complete failed stage=%u att=0x%02x\n",
               static_cast<unsigned>(ble.gatt_stage),
               att_status);
        if (ble.gatt_stage == GattStage::DiscoverServices &&
            ble.service_discovery_retries < MAX_GATT_DISCOVERY_RETRIES) {
            ble.service_discovery_retries++;
            schedule_service_discovery(GATT_DISCOVERY_RETRY_MS);
            return;
        }
        disconnect_and_backoff("GATT query failed");
        return;
    }

    switch (ble.gatt_stage) {
        case GattStage::MtuExchange:
            break;
        case GattStage::DiscoverServices:
            ble.discover_service_index = 0;
            discover_next_service_characteristics();
            break;
        case GattStage::DiscoverCharacteristics:
            ble.discover_service_index++;
            discover_next_service_characteristics();
            break;
        case GattStage::SubscribeAck: {
            if (ble.subscribe_index >= 0 && ble.subscribe_index < static_cast<int>(ble.char_count)) {
                ble.chars[ble.subscribe_index].subscribed = true;
            }
            if (!subscribe_next(Ns2GattRole::AckNotify)) {
                if (!find_char_by_role(Ns2GattRole::AckNotify)) {
                    disconnect_and_backoff("ACK notify characteristic missing");
                } else {
                    start_controller_init();
                }
            }
            break;
        }
        case GattStage::SubscribeInput: {
            if (ble.subscribe_index >= 0 && ble.subscribe_index < static_cast<int>(ble.char_count)) {
                ble.chars[ble.subscribe_index].subscribed = true;
            }
            if (!subscribe_next(Ns2GattRole::InputNotify)) {
                ble.gatt_stage = GattStage::Ready;
                ns2_status_set_state(Ns2BleState::ConnectedNoNotify);
                printf("[NS2 GATT] ready; waiting for live input notify\n");
            }
            break;
        }
        case GattStage::Initializing:
            break;
        case GattStage::Idle:
        case GattStage::Ready:
        default:
            break;
    }
}

void handle_notification(uint16_t value_handle, const uint8_t *value, uint16_t value_len) {
    CharacteristicSlot *slot = find_char_by_value_handle(value_handle);
    if (!slot) {
        printf("[NS2 GATT] notify unknown handle=0x%04x len=%u\n", value_handle, value_len);
        return;
    }

    if (slot->role == Ns2GattRole::AckNotify) {
        if (ble.gatt_stage == GattStage::Initializing) {
            char hex[160];
            format_hex(value, value_len > 48 ? 48 : static_cast<uint8_t>(value_len), hex, sizeof(hex));
            printf("[NS2 GATT] init ACK index=%u len=%u first=0x%02x\n",
                   static_cast<unsigned>(ble.init_index),
                   static_cast<unsigned>(value_len),
                   value_len > 0 ? value[0] : 0);
            if (value_len > 0 && (value[0] == 0x15 || ble.init_index == 3)) {
                printf("[NS2 GATT] init ACK hex=%s%s\n",
                       hex,
                       value_len > 48 ? " ..." : "");
            }
            ble.init_index++;
            send_current_init_command();
        }
        return;
    }

    if (slot->role == Ns2GattRole::InputNotify) {
        Ns2InputReportKind kind = Ns2InputReportKind::Unknown;
        if (ns2_gatt_uuid128_equals(slot->characteristic.uuid128, ADV_UUID_NOTIFY_FD2)) {
            kind = Ns2InputReportKind::Fd2;
        } else if (ns2_gatt_uuid128_equals(slot->characteristic.uuid128, UUID_NOTIFY_LEGACY)) {
            kind = Ns2InputReportKind::Legacy;
        }
        if (!ns2_input_parse_notify(kind, value, value_len)) {
            printf("[NS2 INPUT] parse failed kind=%s len=%u first=0x%02x\n",
                   ns2_input_kind_name(kind),
                   static_cast<unsigned>(value_len),
                   value_len > 0 ? value[0] : 0);
        }
        save_current_target_if_live("live input");
        ns2_status_note_notify(value_len);
        if (ns2_status_get_state() != Ns2BleState::ConnectedLive) {
            printf("[NS2 BLE] live input notify started len=%u count=%lu\n",
                   static_cast<unsigned>(value_len),
                   static_cast<unsigned long>(ns2_status_notify_count()));
        }
        ns2_status_set_state(Ns2BleState::ConnectedLive);
    }
}

void on_connected() {
    ble.connecting = false;
    ble.scanning = false;
    ble.pending_connect_valid = false;
    memcpy(ble.current_addr, ble.pending_addr, 6);
    ble.current_addr_type = ble.pending_addr_type;
    ble.current_addr_valid = true;
    reset_retry_window();
    ns2_status_clear_error();
    ns2_status_note_connected(ble.pending_addr, ble.pending_addr_type);
    request_fast_connection_update("connect");

    bool saved_target_match = false;
    uint8_t saved_addr[6];
    uint8_t saved_type = 0;
    if (ns2_config_has_saved_target()) {
        ns2_config_get_saved_target(saved_addr, &saved_type);
        saved_target_match = ns2_addr_equal(saved_addr, ble.pending_addr) &&
                             saved_type == ble.pending_addr_type;
    }

    if (ble.connect_mode == ConnectMode::PairingScan) {
        ble.security_pending = true;
        ble.security_required = true;
        ble.security_started_ms = now_ms();
        ns2_status_set_state(Ns2BleState::Pairing);
        printf("[NS2 SM] explicit pair mode; requesting BLE bonding\n");
        sm_request_pairing(ble.con_handle);
        return;
    }

    if (saved_target_match) {
        if (le_device_db_count() == 0) {
            printf("[NS2 SM] saved target has no local bond; skipping MTU and using direct GATT reconnect path\n");
            schedule_service_discovery(SAVED_RECONNECT_GATT_DELAY_MS);
            return;
        } else {
            printf("[NS2 GATT] saved target connected with local bond; using direct GATT reconnect path\n");
        }
    }
    schedule_mtu_exchange(GATT_DISCOVERY_DELAY_MS);
}

void on_connection_failed(uint8_t status) {
    printf("[NS2 BLE] connect failed status=0x%02x\n", status);
    ble.connecting = false;
    ble.pending_connect_valid = false;
    char error[32];
    snprintf(error, sizeof(error), "conn fail 0x%02x", status);
    if (ble.connect_mode == ConnectMode::DirectSaved) {
        ns2_status_set_last_error(error);
        start_scan_internal(ConnectMode::ScanAuto);
        return;
    }
    enter_backoff(error);
}

void maybe_connect_from_advertisement(const uint8_t addr[6],
                                      uint8_t addr_type,
                                      int8_t rssi,
                                      bool candidate,
                                      const char *reason) {
    if (!ble.scanning || ble.connecting || ble.con_handle != HCI_CON_HANDLE_INVALID) {
        return;
    }

    bool saved_match = false;
    uint8_t saved_addr[6];
    uint8_t saved_type = 0;
    if (ns2_config_has_saved_target()) {
        ns2_config_get_saved_target(saved_addr, &saved_type);
        saved_match = ns2_addr_equal(saved_addr, addr) && saved_type == addr_type;
    }

    if (ble.connect_mode != ConnectMode::PairingScan && saved_match) {
        start_connect(addr, addr_type, ble.connect_mode, "saved-scan");
        return;
    }

    if (candidate && rssi >= STRONG_CANDIDATE_RSSI) {
        start_connect(addr, addr_type, ble.connect_mode, reason ? reason : "candidate");
    }
}

void handle_advertising_report(uint8_t *packet) {
    bd_addr_t addr;
    gap_event_advertising_report_get_address(packet, addr);
    const uint8_t addr_type = gap_event_advertising_report_get_address_type(packet);
    const int8_t rssi = gap_event_advertising_report_get_rssi(packet);
    const uint8_t data_len = gap_event_advertising_report_get_data_length(packet);
    const uint8_t *data = gap_event_advertising_report_get_data(packet);

    AdvertisementInfo adv;
    parse_advertisement(data, data_len, &adv);

    uint8_t saved_addr[6];
    uint8_t saved_type = 0;
    const bool has_saved = ns2_config_has_saved_target();
    if (has_saved) {
        ns2_config_get_saved_target(saved_addr, &saved_type);
    }
    const bool saved_match = has_saved && ns2_addr_equal(saved_addr, addr) && saved_type == addr_type;
    const bool pair_mode = ble.connect_mode == ConnectMode::PairingScan;
    const bool name_match = name_looks_like_ns2(adv.name);
    bool candidate = false;
    switch (ble.connect_mode) {
        case ConnectMode::PairingScan:
            candidate = adv.ns2_pairing_mfg;
            break;
        case ConnectMode::ScanAuto:
        case ConnectMode::DirectSaved:
            candidate = saved_match;
            break;
        case ConnectMode::ManualScan:
            candidate = adv.ns2_mfg_prefix || adv.service_match;
            break;
        case ConnectMode::None:
        default:
            candidate = false;
            break;
    }

    const char *reason = "none";
    if (pair_mode && adv.ns2_pairing_mfg) {
        reason = "pairing_mfg";
    } else if (pair_mode && adv.ns2_mfg_prefix) {
        reason = "bound_mfg_skip";
    } else if (saved_match) {
        reason = "saved_addr";
    } else if (adv.ns2_mfg_prefix) {
        reason = "ns2_mfg";
    } else if (adv.service_match) {
        reason = "service_uuid";
    } else if (name_match) {
        reason = "name_only";
    } else if (adv.nintendo_mfg) {
        reason = "nintendo_other";
    } else if (adv.appearance_match) {
        reason = "appearance_only";
    }

    if (candidate || adv.nintendo_mfg) {
        ns2_status_note_candidate(addr, addr_type, rssi, adv.name, candidate, reason);
    }

    if ((candidate || adv.nintendo_mfg) && ble.adv_log_count < 40) {
        char formatted[32];
        char mfg_hex[128];
        ns2_format_addr(addr, addr_type, formatted, sizeof(formatted));
        format_hex(adv.nintendo_data, adv.nintendo_data_len, mfg_hex, sizeof(mfg_hex));
        printf("[NS2 BLE] adv addr=%s rssi=%d name=\"%s\" candidate=%u reason=%s mfg=%s\n",
               formatted,
               rssi,
               adv.name[0] ? adv.name : "<none>",
               candidate ? 1u : 0u,
               reason,
               adv.nintendo_mfg ? mfg_hex : "<none>");
        ble.adv_log_count++;
    }

    maybe_connect_from_advertisement(addr, addr_type, rssi, candidate, reason);
}

void hci_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    (void)channel;
    (void)size;

    if (packet_type != HCI_EVENT_PACKET) {
        return;
    }

    const uint8_t event_type = hci_event_packet_get_type(packet);
    switch (event_type) {
        case BTSTACK_EVENT_STATE: {
            const uint8_t state = btstack_event_state_get_state(packet);
            printf("[NS2 HCI] state=%u\n", state);
            if (state == HCI_STATE_WORKING) {
                ble.hci_ready = true;
                bd_addr_t local_addr;
                gap_local_bd_addr(local_addr);
                ns2_gatt_set_console_mac(local_addr);
                ns2_status_set_local_addr(local_addr);
                printf("[NS2 HCI] local addr=%s\n", bd_addr_to_str(local_addr));
                print_le_db_summary();
                ns2_status_set_state(Ns2BleState::Idle);
                ble.auto_start_due_ms = now_ms() + AUTO_START_DELAY_MS;
            }
            break;
        }

        case GAP_EVENT_ADVERTISING_REPORT:
            handle_advertising_report(packet);
            break;

        case HCI_EVENT_LE_META: {
            const uint8_t subevent = hci_event_le_meta_get_subevent_code(packet);
            if (subevent == HCI_SUBEVENT_LE_CONNECTION_COMPLETE) {
                const uint8_t status = hci_subevent_le_connection_complete_get_status(packet);
                if (status == ERROR_CODE_SUCCESS) {
                    ble.con_handle = hci_subevent_le_connection_complete_get_connection_handle(packet);
                    printf("[NS2 HCI] connected handle=0x%04x interval=%u latency=%u supervision=%u\n",
                           ble.con_handle,
                           hci_subevent_le_connection_complete_get_conn_interval(packet),
                           hci_subevent_le_connection_complete_get_conn_latency(packet),
                           hci_subevent_le_connection_complete_get_supervision_timeout(packet));
                    on_connected();
                } else {
                    on_connection_failed(status);
                }
            } else if (subevent == HCI_SUBEVENT_LE_CONNECTION_UPDATE_COMPLETE) {
                printf("[NS2 HCI] conn update complete status=0x%02x handle=0x%04x interval=%u latency=%u supervision=%u\n",
                       hci_subevent_le_connection_update_complete_get_status(packet),
                       hci_subevent_le_connection_update_complete_get_connection_handle(packet),
                       hci_subevent_le_connection_update_complete_get_conn_interval(packet),
                       hci_subevent_le_connection_update_complete_get_conn_latency(packet),
                       hci_subevent_le_connection_update_complete_get_supervision_timeout(packet));
            }
            break;
        }

        case HCI_EVENT_DISCONNECTION_COMPLETE: {
            const uint8_t reason = hci_event_disconnection_complete_get_reason(packet);
            printf("[NS2 HCI] disconnected reason=0x%02x\n", reason);
            ble.con_handle = HCI_CON_HANDLE_INVALID;
            ble.connecting = false;
            ble.scanning = false;
            ble.current_addr_valid = false;
            ble.security_pending = false;
            ble.security_required = false;
            ble.security_started_ms = 0;
            ns2_status_note_disconnected(reason);
            if (ble.pair_required_after_disconnect) {
                ble.pair_required_after_disconnect = false;
                ble.connect_mode = ConnectMode::None;
                reset_gatt_cache();
                ns2_status_set_last_error("pair required");
                ns2_status_set_state(Ns2BleState::Error);
                printf("[NS2 BLE] auto reconnect stopped; pair required\n");
            } else {
                char error[32];
                snprintf(error, sizeof(error), "drop 0x%02x", reason);
                ns2_status_set_state(Ns2BleState::Disconnected);
                enter_backoff(error);
            }
            break;
        }

        case GAP_EVENT_RSSI_MEASUREMENT: {
            const hci_con_handle_t handle = gap_event_rssi_measurement_get_con_handle(packet);
            if (handle == ble.con_handle) {
                ns2_status_set_rssi(static_cast<int8_t>(gap_event_rssi_measurement_get_rssi(packet)));
            }
            break;
        }

        default:
            break;
    }
}

void gatt_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    (void)channel;
    (void)size;

    if (packet_type != HCI_EVENT_PACKET) {
        return;
    }

    const uint8_t event_type = hci_event_packet_get_type(packet);
    switch (event_type) {
        case GATT_EVENT_SERVICE_QUERY_RESULT: {
            if (ble.service_count >= MAX_SERVICES) {
                printf("[NS2 GATT] service cache full\n");
                break;
            }
            ServiceSlot &slot = ble.services[ble.service_count++];
            slot.used = true;
            gatt_event_service_query_result_get_service(packet, &slot.service);
            printf("[NS2 GATT] service start=0x%04x end=0x%04x\n",
                   slot.service.start_group_handle,
                   slot.service.end_group_handle);
            break;
        }

        case GATT_EVENT_CHARACTERISTIC_QUERY_RESULT: {
            if (ble.char_count >= MAX_CHARS) {
                printf("[NS2 GATT] characteristic cache full\n");
                break;
            }
            CharacteristicSlot &slot = ble.chars[ble.char_count++];
            memset(&slot, 0, sizeof(slot));
            slot.used = true;
            gatt_event_characteristic_query_result_get_characteristic(packet, &slot.characteristic);
            slot.role = ns2_gatt_classify_uuid(slot.characteristic.uuid16, slot.characteristic.uuid128);
            if (slot.role == Ns2GattRole::Command) {
                ble.command_value_handle = slot.characteristic.value_handle;
                ble.command_write_without_response =
                    (slot.characteristic.properties & ATT_PROPERTY_WRITE_WITHOUT_RESPONSE) != 0;
            }
            if (slot.role == Ns2GattRole::Rumble) {
                ble.rumble_value_handle = slot.characteristic.value_handle;
                ble.rumble_write_without_response =
                    (slot.characteristic.properties & ATT_PROPERTY_WRITE_WITHOUT_RESPONSE) != 0;
            }
            char uuid[48];
            ns2_gatt_format_uuid(slot.characteristic.uuid16, slot.characteristic.uuid128, uuid, sizeof(uuid));
            printf("[NS2 GATT] char value=0x%04x props=0x%02x role=%s uuid=%s\n",
                   slot.characteristic.value_handle,
                   slot.characteristic.properties,
                   ns2_gatt_role_name(slot.role),
                   uuid);
            break;
        }

        case GATT_EVENT_QUERY_COMPLETE:
            handle_gatt_query_complete(gatt_event_query_complete_get_att_status(packet));
            break;

        case GATT_EVENT_MTU: {
            const hci_con_handle_t handle = gatt_event_mtu_get_handle(packet);
            const uint16_t mtu = gatt_event_mtu_get_MTU(packet);
            printf("[NS2 GATT] MTU exchanged handle=0x%04x mtu=%u\n", handle, mtu);
            if (handle == ble.con_handle && ble.gatt_stage == GattStage::MtuExchange) {
                ble.mtu_exchange_pending = false;
                ble.mtu_exchange_started_ms = 0;
                start_service_discovery();
            }
            break;
        }

        case GATT_EVENT_NOTIFICATION: {
            const uint16_t value_handle = gatt_event_notification_get_value_handle(packet);
            const uint16_t value_len = gatt_event_notification_get_value_length(packet);
            const uint8_t *value = gatt_event_notification_get_value(packet);
            handle_notification(value_handle, value, value_len);
            break;
        }

        default:
            break;
    }
}

void sm_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    (void)channel;
    (void)size;

    if (packet_type != HCI_EVENT_PACKET) {
        return;
    }

    bd_addr_t addr;
    char formatted[32];
    const uint8_t event_type = hci_event_packet_get_type(packet);
    switch (event_type) {
        case SM_EVENT_JUST_WORKS_REQUEST:
            printf("[NS2 SM] just works request; confirm\n");
            sm_just_works_confirm(sm_event_just_works_request_get_handle(packet));
            break;
        case SM_EVENT_NUMERIC_COMPARISON_REQUEST:
            printf("[NS2 SM] numeric comparison request; confirm passkey=%lu\n",
                   static_cast<unsigned long>(sm_event_numeric_comparison_request_get_passkey(packet)));
            sm_numeric_comparison_confirm(sm_event_numeric_comparison_request_get_handle(packet));
            break;
        case SM_EVENT_PASSKEY_DISPLAY_NUMBER:
            printf("[NS2 SM] passkey display=%lu\n",
                   static_cast<unsigned long>(sm_event_passkey_display_number_get_passkey(packet)));
            break;
        case SM_EVENT_PASSKEY_INPUT_NUMBER:
            printf("[NS2 SM] passkey input requested; cannot input on dongle\n");
            break;
        case SM_EVENT_IDENTITY_CREATED:
            sm_event_identity_created_get_identity_address(packet, addr);
            ns2_format_addr(addr,
                            sm_event_identity_created_get_identity_addr_type(packet),
                            formatted,
                            sizeof(formatted));
            printf("[NS2 SM] identity created index=%u addr=%s\n",
                   static_cast<unsigned>(sm_event_identity_created_get_index(packet)),
                   formatted);
            break;
        case SM_EVENT_IDENTITY_RESOLVING_SUCCEEDED:
            sm_event_identity_resolving_succeeded_get_identity_address(packet, addr);
            ns2_format_addr(addr,
                            sm_event_identity_resolving_succeeded_get_identity_addr_type(packet),
                            formatted,
                            sizeof(formatted));
            printf("[NS2 SM] identity resolved index=%u addr=%s\n",
                   static_cast<unsigned>(sm_event_identity_resolving_succeeded_get_index(packet)),
                   formatted);
            break;
        case SM_EVENT_IDENTITY_RESOLVING_FAILED:
            printf("[NS2 SM] identity resolving failed\n");
            break;
        case SM_EVENT_PAIRING_STARTED:
            printf("[NS2 SM] pairing started\n");
            break;
        case SM_EVENT_PAIRING_COMPLETE:
            printf("[NS2 SM] pairing complete status=0x%02x reason=0x%02x\n",
                   sm_event_pairing_complete_get_status(packet),
                   sm_event_pairing_complete_get_reason(packet));
            print_le_db_summary();
            if (sm_event_pairing_complete_get_status(packet) == ERROR_CODE_SUCCESS &&
                le_device_db_count() == 0) {
                printf("[NS2 SM] pairing succeeded without a standard BLE bond entry\n");
            }
            if (ble.security_pending) {
                ble.security_pending = false;
                ble.security_started_ms = 0;
                if (ble.con_handle != HCI_CON_HANDLE_INVALID) {
                    if (sm_event_pairing_complete_get_status(packet) != ERROR_CODE_SUCCESS) {
                        ns2_status_set_last_error("pairing failed");
                        if (ble.security_required) {
                            ble.security_required = false;
                            clear_saved_target_after_security_failure();
                            disconnect_and_backoff("pairing failed");
                            return;
                        }
                    }
                    ble.security_required = false;
                    schedule_mtu_exchange(GATT_DISCOVERY_DELAY_MS);
                }
            }
            break;
        case SM_EVENT_REENCRYPTION_STARTED:
            sm_event_reencryption_started_get_address(packet, addr);
            ns2_format_addr(addr,
                            sm_event_reencryption_started_get_addr_type(packet),
                            formatted,
                            sizeof(formatted));
            printf("[NS2 SM] re-encryption started addr=%s\n", formatted);
            break;
        case SM_EVENT_REENCRYPTION_COMPLETE:
            sm_event_reencryption_complete_get_address(packet, addr);
            ns2_format_addr(addr,
                            sm_event_reencryption_complete_get_addr_type(packet),
                            formatted,
                            sizeof(formatted));
            printf("[NS2 SM] re-encryption complete status=0x%02x addr=%s\n",
                   sm_event_reencryption_complete_get_status(packet),
                   formatted);
            print_le_db_summary();
            if (sm_event_reencryption_complete_get_status(packet) == ERROR_CODE_PIN_OR_KEY_MISSING) {
                printf("[NS2 SM] key missing; deleting local bond and waiting for controller-side pairing\n");
                gap_delete_bonding(static_cast<bd_addr_type_t>(sm_event_reencryption_complete_get_addr_type(packet)), addr);
                ble.security_pending = false;
                ble.security_required = false;
                ble.security_started_ms = 0;
            } else if (ble.security_pending) {
                ble.security_pending = false;
                ble.security_started_ms = 0;
                if (ble.con_handle != HCI_CON_HANDLE_INVALID) {
                    ble.security_required = false;
                    schedule_mtu_exchange(GATT_DISCOVERY_DELAY_MS);
                }
            }
            break;
        default:
            break;
    }
}

} // namespace

void ns2_ble_init() {
    memset(&ble, 0, sizeof(ble));
    ble.con_handle = HCI_CON_HANDLE_INVALID;
    ble.connect_mode = ConnectMode::None;
    reset_gatt_cache();

    ns2_config_load();
    ns2_status_set_state(Ns2BleState::BleInit);

    l2cap_init();
    l2cap_set_max_le_mtu(NS2_LE_MTU);
    sm_init();
    sm_set_io_capabilities(IO_CAPABILITY_NO_INPUT_NO_OUTPUT);
    sm_set_authentication_requirements(SM_AUTHREQ_BONDING | SM_AUTHREQ_SECURE_CONNECTION);
    printf("[NS2 SM] auth req=bonding+secure-connections io=no-input-no-output\n");
    gatt_client_init();
    gatt_client_mtu_enable_auto_negotiation(0);
    configure_fast_connection_parameters();
    printf("[NS2 HCI] fast conn defaults scan=%u/%u interval=%u..%u latency=%u supervision=%u\n",
           SCAN_INTERVAL,
           SCAN_WINDOW,
           FAST_CONN_INTERVAL_MIN,
           FAST_CONN_INTERVAL_MAX,
           FAST_CONN_LATENCY,
           FAST_CONN_SUPERVISION_TIMEOUT);
    printf("[NS2 GATT] LE MTU capped at %u\n", NS2_LE_MTU);

    hci_event_callback_registration.callback = &hci_packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    sm_event_callback_registration.callback = &sm_packet_handler;
    sm_add_event_handler(&sm_event_callback_registration);

    hci_power_control(HCI_POWER_ON);
}

void ns2_ble_tick() {
    const uint64_t now = now_ms();

    if (ble.hci_ready &&
        ns2_config_auto_connect_enabled() &&
        ble.auto_start_due_ms != 0 &&
        now >= ble.auto_start_due_ms &&
        ns2_status_get_state() == Ns2BleState::Idle) {
        ble.auto_start_due_ms = 0;
        start_auto_attempt();
    }

    if (ns2_status_get_state() == Ns2BleState::Backoff &&
        ns2_config_auto_connect_enabled() &&
        ble.next_retry_ms != 0 &&
        now >= ble.next_retry_ms) {
        start_auto_attempt();
    }

    if (ble.con_handle != HCI_CON_HANDLE_INVALID &&
        ble.mtu_exchange_due_ms != 0 &&
        now >= ble.mtu_exchange_due_ms) {
        ble.mtu_exchange_due_ms = 0;
        start_mtu_exchange();
    }

    if (ble.con_handle != HCI_CON_HANDLE_INVALID &&
        ble.mtu_exchange_pending &&
        ble.mtu_exchange_started_ms != 0 &&
        now - ble.mtu_exchange_started_ms > MTU_EXCHANGE_TIMEOUT_MS) {
        printf("[NS2 GATT] MTU exchange timeout; continuing with service discovery\n");
        ble.mtu_exchange_pending = false;
        ble.mtu_exchange_started_ms = 0;
        start_service_discovery();
    }

    if (ble.con_handle != HCI_CON_HANDLE_INVALID &&
        ble.service_discovery_due_ms != 0 &&
        now >= ble.service_discovery_due_ms) {
        start_service_discovery();
    }

    if (ble.con_handle != HCI_CON_HANDLE_INVALID &&
        ble.known_handle_due_ms != 0 &&
        now >= ble.known_handle_due_ms) {
        ble.known_handle_due_ms = 0;
        start_known_handle_flow();
    }

    if (ble.con_handle != HCI_CON_HANDLE_INVALID &&
        ble.security_pending &&
        ble.security_started_ms != 0 &&
        now - ble.security_started_ms > SECURITY_WAIT_MS) {
        printf("[NS2 SM] security wait timeout required=%u\n", ble.security_required ? 1u : 0u);
        ble.security_pending = false;
        ble.security_started_ms = 0;
        if (ble.security_required) {
            ble.security_required = false;
            clear_saved_target_after_security_failure();
            disconnect_and_backoff("security timeout");
            return;
        }
        schedule_mtu_exchange(GATT_DISCOVERY_DELAY_MS);
    }

    if (ble.connecting &&
        ble.connect_started_ms != 0 &&
        now - ble.connect_started_ms > CONNECT_WATCHDOG_MS) {
        printf("[NS2 BLE] connect watchdog expired\n");
        gap_connect_cancel();
        ble.connecting = false;
        enter_backoff("connect timeout");
    }

    if (ble.gatt_stage == GattStage::Initializing &&
        ble.init_command_sent_ms != 0 &&
        now - ble.init_command_sent_ms > INIT_ACK_TIMEOUT_MS) {
        disconnect_and_backoff("controller init ACK timeout");
    }

    if (ble.con_handle != HCI_CON_HANDLE_INVALID &&
        now - ble.last_rssi_poll_ms >= RSSI_POLL_MS) {
        ble.last_rssi_poll_ms = now;
        gap_read_rssi(ble.con_handle);
    }

    const Ns2BleState state = ns2_status_get_state();
    if ((state == Ns2BleState::ConnectedNoNotify || state == Ns2BleState::ConnectedLive) &&
        !ns2_status_live_notify()) {
        const uint32_t age = ns2_status_last_notify_age_ms();
        if (age != UINT32_MAX && age > NOTIFY_TIMEOUT_MS) {
            disconnect_and_backoff("notify timeout");
        }
    }
}

void ns2_ble_start_scan() {
    ble.direct_saved_tried = true;
    start_scan_internal(ConnectMode::ManualScan);
}

void ns2_ble_pair() {
    if (ble.scanning) {
        gap_stop_scan();
        ble.scanning = false;
    }
    if (ble.connecting) {
        gap_connect_cancel();
        ble.connecting = false;
    }
    if (ble.con_handle != HCI_CON_HANDLE_INVALID) {
        gap_disconnect(ble.con_handle);
        ns2_status_set_last_error("disconnect first, retry pair");
        return;
    }
    ble.direct_saved_tried = true;
    reset_retry_window();
    start_scan_internal(ConnectMode::PairingScan);
}

void ns2_ble_reconnect() {
    if (ble.scanning) {
        gap_stop_scan();
        ble.scanning = false;
    }
    if (ble.con_handle != HCI_CON_HANDLE_INVALID) {
        gap_disconnect(ble.con_handle);
    }
    ble.direct_saved_tried = false;
    reset_retry_window();
    ns2_status_set_state(Ns2BleState::Idle);
    start_auto_attempt();
}

void ns2_ble_disconnect() {
    if (ble.scanning) {
        gap_stop_scan();
        ble.scanning = false;
    }
    if (ble.connecting) {
        gap_connect_cancel();
        ble.connecting = false;
    }
    if (ble.con_handle != HCI_CON_HANDLE_INVALID) {
        gap_disconnect(ble.con_handle);
    } else {
        ns2_status_clear_connection();
        ns2_status_set_state(Ns2BleState::Idle);
    }
}

void ns2_ble_forget() {
    uint8_t saved_addr[6];
    uint8_t saved_type = 0;
    if (ns2_config_has_saved_target()) {
        ns2_config_get_saved_target(saved_addr, &saved_type);
        gap_delete_bonding(static_cast<bd_addr_type_t>(saved_type), saved_addr);
    }
    ns2_config_forget_saved_target();
    (void)ns2_config_save();
    ble.direct_saved_tried = false;
}

void ns2_ble_set_auto_connect(bool enabled) {
    ns2_config_set_auto_connect(enabled);
    (void)ns2_config_save();
    if (enabled) {
        ble.auto_start_due_ms = now_ms() + 100;
    } else if (ns2_status_get_state() == Ns2BleState::Error ||
               ns2_status_get_state() == Ns2BleState::Backoff) {
        ns2_status_clear_error();
        ns2_status_set_state(Ns2BleState::Idle);
    }
}

bool ns2_ble_auto_connect_enabled() {
    return ns2_config_auto_connect_enabled();
}

bool ns2_ble_rumble_ready() {
    return ble.con_handle != HCI_CON_HANDLE_INVALID && ble.rumble_value_handle != 0;
}

bool ns2_ble_send_rumble(const uint8_t *data, uint16_t len) {
    if (!data || len == 0 || !ns2_ble_rumble_ready()) {
        return false;
    }

    uint8_t status;
    if (ble.rumble_write_without_response) {
        status = gatt_client_write_value_of_characteristic_without_response(
            ble.con_handle,
            ble.rumble_value_handle,
            len,
            const_cast<uint8_t *>(data));
    } else {
        status = gatt_client_write_value_of_characteristic(
            gatt_packet_handler,
            ble.con_handle,
            ble.rumble_value_handle,
            len,
            const_cast<uint8_t *>(data));
    }
    return status == ERROR_CODE_SUCCESS;
}
