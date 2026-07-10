#include "ns2_ble.h"

#include "ns2_config.h"
#include "ns2_gatt.h"
#include "ns2_input.h"
#include "ns2_protocol.h"

#include <algorithm>
#include <cstring>

#include "esp_check.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_hs.h"
#include "host/ble_hs_adv.h"
#include "host/ble_sm.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "nvs_flash.h"
#include "os/os_mbuf.h"
#include "services/gap/ble_svc_gap.h"

extern "C" void ble_store_config_init(void);

namespace ns2 {
namespace {

const char *TAG = "ns2-ble";

constexpr size_t kMaxServices = 24;
constexpr size_t kMaxChars = 72;
constexpr uint16_t kKnownInputFd2Handle = 0x000a;
constexpr uint16_t kKnownCommandHandle = 0x0014;
constexpr uint16_t kKnownAckHandle = 0x001a;
constexpr int8_t kConnectRssiMin = -82;
constexpr uint32_t kConnectTimeoutMs = 30000;
constexpr uint64_t kConnectTimeoutUs = 30000000ULL;
constexpr uint64_t kInitAckTimeoutUs = 3500000ULL;
constexpr uint64_t kNotifyStaleLogUs = 60000000ULL;
constexpr uint64_t kCommandSpacingUs = 1200ULL;
constexpr uint64_t kConnectRetryBaseDelayUs = 1200000ULL;
constexpr uint64_t kConnectRetryMaxDelayUs = 10000000ULL;
constexpr uint64_t kReconnectAfterReadyDelayUs = 900000ULL;
constexpr uint16_t kCccdNotify = 0x0001;
constexpr uint16_t kFastConnIntervalMin = 0x000a;
constexpr uint16_t kFastConnIntervalMax = 0x000c;
constexpr uint16_t kFastConnLatency = 0;
constexpr uint16_t kFastConnSupervisionTimeout = 400;

enum class Stage : uint8_t {
    Idle,
    Scanning,
    Connecting,
    DiscoverServices,
    DiscoverCharacteristics,
    SubscribeAck,
    Initializing,
    SubscribeInput,
    Ready,
};

struct ServiceSlot {
    uint16_t start = 0;
    uint16_t end = 0;
};

struct CharSlot {
    bool used = false;
    uint16_t value_handle = 0;
    uint16_t end_handle = 0;
    uint8_t properties = 0;
    uint8_t uuid_be[kUuid128Len] = {};
    GattRole role = GattRole::Other;
    bool subscribed = false;
};

struct Runtime {
    bool started = false;
    uint8_t own_addr_type = 0;
    uint16_t conn_handle = BLE_HS_CONN_HANDLE_NONE;
    Stage stage = Stage::Idle;
    BleStats stats = {};
    ble_addr_t pending_addr = {};
    ServiceSlot services[kMaxServices] = {};
    CharSlot chars[kMaxChars] = {};
    size_t service_count = 0;
    size_t char_count = 0;
    size_t discover_service_index = 0;
    int subscribe_index = -1;
    uint8_t init_index = 0;
    uint16_t command_handle = 0;
    uint16_t rumble_handle = 0;
    bool command_no_rsp = false;
    bool rumble_no_rsp = false;
    bool pending_subscribe_ack = false;
    bool pending_subscribe_input = false;
    bool pair_mode = false;
    bool manual_scan = false;
    bool stay_disconnected = false;
    bool target_saved_this_connection = false;
    uint64_t connect_started_us = 0;
    uint64_t next_connect_allowed_us = 0;
    uint64_t scan_restart_due_us = 0;
    uint64_t init_command_sent_us = 0;
    uint64_t stage_started_us = 0;
    uint64_t next_gatt_action_us = 0;
    uint32_t ack_notify_logged = 0;
    uint32_t input_notify_logged = 0;
    uint32_t input_parse_failed_logged = 0;
    uint64_t notify_sample_us = 0;
    uint32_t notify_sample_count = 0;
    uint64_t notify_last_us = 0;
    uint64_t next_notify_stale_log_us = 0;
    uint8_t consecutive_connect_failures = 0;
};

Runtime s_ble;

void start_scan();
void stop_scan();
void schedule_scan(uint64_t delay_us);
void disconnect_current(const char *reason);
int gap_event(ble_gap_event *event, void *arg);
int service_cb(uint16_t conn_handle, const ble_gatt_error *error, const ble_gatt_svc *service, void *arg);
int characteristic_cb(uint16_t conn_handle, const ble_gatt_error *error, const ble_gatt_chr *chr, void *arg);
int write_cb(uint16_t conn_handle, const ble_gatt_error *error, ble_gatt_attr *attr, void *arg);
int mtu_cb(uint16_t conn_handle, const ble_gatt_error *error, uint16_t mtu, void *arg);
void send_current_init_command();

void uuid_to_be(const ble_uuid_t *uuid, uint8_t out[kUuid128Len]) {
    std::memset(out, 0, kUuid128Len);
    if (uuid == nullptr || uuid->type != BLE_UUID_TYPE_128) {
        return;
    }
    const ble_uuid128_t *uuid128 = reinterpret_cast<const ble_uuid128_t *>(uuid);
    for (size_t i = 0; i < kUuid128Len; ++i) {
        out[i] = uuid128->value[kUuid128Len - 1 - i];
    }
}

void update_ready_flags() {
    s_ble.stats.connected = s_ble.conn_handle != BLE_HS_CONN_HANDLE_NONE;
    s_ble.stats.gatt_ready = s_ble.stage == Stage::Ready;
    s_ble.stats.rumble_ready = s_ble.rumble_handle != 0;
    s_ble.stats.services = static_cast<uint32_t>(s_ble.service_count);
    s_ble.stats.characteristics = static_cast<uint32_t>(s_ble.char_count);
}

void update_conn_desc() {
    if (s_ble.conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        s_ble.stats.conn_interval_units = 0;
        s_ble.stats.conn_interval_us = 0;
        s_ble.stats.conn_latency = 0;
        s_ble.stats.conn_supervision_timeout = 0;
        return;
    }
    ble_gap_conn_desc desc{};
    const int rc = ble_gap_conn_find(s_ble.conn_handle, &desc);
    if (rc != 0) {
        s_ble.stats.last_error = rc;
        return;
    }
    s_ble.stats.conn_interval_units = desc.conn_itvl;
    s_ble.stats.conn_interval_us = static_cast<uint32_t>(desc.conn_itvl) * 1250UL;
    s_ble.stats.conn_latency = desc.conn_latency;
    s_ble.stats.conn_supervision_timeout = desc.supervision_timeout;
    uint8_t tx_phy = 0;
    uint8_t rx_phy = 0;
    if (ble_gap_read_le_phy(s_ble.conn_handle, &tx_phy, &rx_phy) == 0) {
        s_ble.stats.tx_phy = tx_phy;
        s_ble.stats.rx_phy = rx_phy;
    }
}

void request_fast_link_params(const char *reason) {
    if (s_ble.conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        return;
    }
    const int phy_rc = ble_gap_set_prefered_le_phy(s_ble.conn_handle,
                                                   BLE_GAP_LE_PHY_2M_MASK,
                                                   BLE_GAP_LE_PHY_2M_MASK,
                                                   BLE_GAP_LE_PHY_CODED_ANY);
    s_ble.stats.phy_update_rc = phy_rc;
    if (phy_rc != 0) {
        s_ble.stats.last_error = phy_rc;
    }

    const int data_rc = ble_gap_set_data_len(s_ble.conn_handle, 251, 2120);
    s_ble.stats.data_len_update_rc = data_rc;
    if (data_rc != 0) {
        s_ble.stats.last_error = data_rc;
    }

    ESP_LOGW(TAG,
             "fast link params reason=%s phy2m_rc=%d data_len_rc=%d",
             reason ? reason : "unknown",
             phy_rc,
             data_rc);
}

void request_fast_connection_update(const char *reason) {
    if (s_ble.conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        return;
    }
    ble_gap_upd_params params{};
    params.itvl_min = kFastConnIntervalMin;
    params.itvl_max = kFastConnIntervalMax;
    params.latency = kFastConnLatency;
    params.supervision_timeout = kFastConnSupervisionTimeout;
    params.min_ce_len = 0;
    params.max_ce_len = 0;
    s_ble.stats.conn_update_requests++;
    const int rc = ble_gap_update_params(s_ble.conn_handle, &params);
    s_ble.stats.conn_update_start_rc = rc;
    if (rc != 0) {
        s_ble.stats.last_error = rc;
    }
    ESP_LOGW(TAG,
             "fast conn update reason=%s interval=%u..%u latency=%u supervision=%u rc=%d",
             reason ? reason : "unknown",
             static_cast<unsigned>(params.itvl_min),
             static_cast<unsigned>(params.itvl_max),
             static_cast<unsigned>(params.latency),
             static_cast<unsigned>(params.supervision_timeout),
             rc);
}

void mark_stage(Stage stage) {
    s_ble.stage = stage;
    s_ble.stage_started_us = esp_timer_get_time();
    update_ready_flags();
}

void clear_gatt() {
    std::memset(s_ble.services, 0, sizeof(s_ble.services));
    std::memset(s_ble.chars, 0, sizeof(s_ble.chars));
    s_ble.service_count = 0;
    s_ble.char_count = 0;
    s_ble.discover_service_index = 0;
    s_ble.subscribe_index = -1;
    s_ble.init_index = 0;
    s_ble.command_handle = 0;
    s_ble.rumble_handle = 0;
    s_ble.command_no_rsp = false;
    s_ble.rumble_no_rsp = false;
    s_ble.pending_subscribe_ack = false;
    s_ble.pending_subscribe_input = false;
    s_ble.target_saved_this_connection = false;
    s_ble.init_command_sent_us = 0;
    s_ble.next_gatt_action_us = 0;
    s_ble.ack_notify_logged = 0;
    s_ble.input_notify_logged = 0;
    s_ble.input_parse_failed_logged = 0;
    s_ble.notify_sample_us = 0;
    s_ble.notify_sample_count = 0;
    s_ble.notify_last_us = 0;
    s_ble.stats.notify_hz = 0;
    s_ble.stats.notify_last_gap_us = 0;
    s_ble.stats.notify_max_gap_us = 0;
    s_ble.stats.input_subscribed = false;
    update_ready_flags();
}

void note_input_notify_rate() {
    const uint64_t now = esp_timer_get_time();
    s_ble.stats.notify_count++;
    if (s_ble.notify_last_us != 0) {
        const uint32_t gap = static_cast<uint32_t>(now - s_ble.notify_last_us);
        s_ble.stats.notify_last_gap_us = gap;
        if (gap > s_ble.stats.notify_max_gap_us) {
            s_ble.stats.notify_max_gap_us = gap;
        }
    }
    s_ble.notify_last_us = now;
    if (s_ble.notify_sample_us == 0) {
        s_ble.notify_sample_us = now;
        s_ble.notify_sample_count = s_ble.stats.notify_count;
        return;
    }
    const uint64_t elapsed = now - s_ble.notify_sample_us;
    if (elapsed >= 1000000ULL) {
        const uint32_t delta = s_ble.stats.notify_count - s_ble.notify_sample_count;
        s_ble.stats.notify_hz = static_cast<uint32_t>((static_cast<uint64_t>(delta) * 1000000ULL) / elapsed);
        s_ble.notify_sample_us = now;
        s_ble.notify_sample_count = s_ble.stats.notify_count;
        s_ble.stats.notify_max_gap_us = 0;
    }
}

uint64_t connect_retry_delay_us() {
    const uint8_t failures = std::min<uint8_t>(s_ble.consecutive_connect_failures, 4);
    uint64_t delay = kConnectRetryBaseDelayUs << failures;
    return std::min<uint64_t>(delay, kConnectRetryMaxDelayUs);
}

CharSlot *find_char_by_handle(uint16_t value_handle) {
    for (size_t i = 0; i < s_ble.char_count; ++i) {
        if (s_ble.chars[i].used && s_ble.chars[i].value_handle == value_handle) {
            return &s_ble.chars[i];
        }
    }
    return nullptr;
}

bool has_role(GattRole role) {
    for (size_t i = 0; i < s_ble.char_count; ++i) {
        if (s_ble.chars[i].used && s_ble.chars[i].role == role) {
            return true;
        }
    }
    return false;
}

bool has_minimum_ns2_gatt() {
    return s_ble.command_handle != 0 &&
           has_role(GattRole::AckNotify) &&
           has_role(GattRole::InputNotify);
}

void add_known_fallback_chars() {
    if (s_ble.char_count != 0) {
        return;
    }
    auto add = [](uint16_t handle, const uint8_t *uuid, GattRole role, uint8_t props) {
        CharSlot &slot = s_ble.chars[s_ble.char_count++];
        slot.used = true;
        slot.value_handle = handle;
        slot.end_handle = static_cast<uint16_t>(handle + 1);
        slot.properties = props;
        if (uuid != nullptr) {
            std::memcpy(slot.uuid_be, uuid, kUuid128Len);
        }
        slot.role = role;
    };
    add(kKnownAckHandle, gatt_fd2_uuid(), GattRole::AckNotify, BLE_GATT_CHR_F_NOTIFY);
    std::memcpy(s_ble.chars[0].uuid_be, "\xc7\x65\xa9\x61\xd9\xd8\x4d\x36\xa2\x0a\x53\x15\xb1\x11\x83\x6a", kUuid128Len);
    add(kKnownCommandHandle, nullptr, GattRole::Command, BLE_GATT_CHR_F_WRITE_NO_RSP | BLE_GATT_CHR_F_WRITE);
    std::memcpy(s_ble.chars[1].uuid_be, "\x64\x9d\x4a\xc9\x8e\xb7\x4e\x6c\xaf\x44\x1e\xa5\x4f\xe5\xf0\x05", kUuid128Len);
    add(kKnownInputFd2Handle, gatt_fd2_uuid(), GattRole::InputNotify, BLE_GATT_CHR_F_NOTIFY);
    s_ble.command_handle = kKnownCommandHandle;
    s_ble.command_no_rsp = true;
}

void start_service_discovery() {
    clear_gatt();
    mark_stage(Stage::DiscoverServices);
    const int rc = ble_gattc_disc_all_svcs(s_ble.conn_handle, service_cb, nullptr);
    if (rc != 0) {
        s_ble.stats.last_error = rc;
        ESP_LOGE(TAG, "service discovery start failed rc=%d", rc);
        add_known_fallback_chars();
        mark_stage(Stage::SubscribeAck);
        s_ble.pending_subscribe_ack = true;
    }
}

void discover_next_characteristics() {
    while (s_ble.discover_service_index < s_ble.service_count) {
        ServiceSlot &svc = s_ble.services[s_ble.discover_service_index];
        if (svc.end <= svc.start) {
            s_ble.discover_service_index++;
            continue;
        }
        mark_stage(Stage::DiscoverCharacteristics);
        const int rc = ble_gattc_disc_all_chrs(s_ble.conn_handle,
                                               static_cast<uint16_t>(svc.start + 1),
                                               svc.end,
                                               characteristic_cb,
                                               nullptr);
        if (rc == 0) {
            return;
        }
        ESP_LOGW(TAG, "characteristic discovery start failed rc=%d svc=%u", rc, static_cast<unsigned>(s_ble.discover_service_index));
        s_ble.discover_service_index++;
    }
    mark_stage(Stage::SubscribeAck);
    s_ble.subscribe_index = -1;
    s_ble.pending_subscribe_ack = true;
}

void subscribe_role(GattRole role) {
    for (int i = s_ble.subscribe_index + 1; i < static_cast<int>(s_ble.char_count); ++i) {
        CharSlot &slot = s_ble.chars[i];
        if (!slot.used || slot.role != role || slot.subscribed) {
            continue;
        }
        if (role == GattRole::InputNotify && !gatt_uuid128_equals(slot.uuid_be, gatt_fd2_uuid())) {
            continue;
        }
        s_ble.subscribe_index = i;
        const uint8_t value[2] = {static_cast<uint8_t>(kCccdNotify & 0xff), static_cast<uint8_t>(kCccdNotify >> 8)};
        const uint16_t cccd = static_cast<uint16_t>(slot.value_handle + 1);
        const int rc = ble_gattc_write_flat(s_ble.conn_handle, cccd, value, sizeof(value), write_cb, reinterpret_cast<void *>(static_cast<uintptr_t>(role)));
        ESP_LOGI(TAG, "subscribe role=%s value=0x%04x cccd=0x%04x rc=%d", gatt_role_name(role), slot.value_handle, cccd, rc);
        if (rc == 0) {
            return;
        }
        s_ble.stats.last_error = rc;
    }

    if (role == GattRole::AckNotify) {
        mark_stage(Stage::Initializing);
        s_ble.init_index = 0;
        ESP_LOGI(TAG, "ack subscription complete; starting controller init");
        send_current_init_command();
    } else {
        mark_stage(Stage::Ready);
        s_ble.stats.input_subscribed = true;
        ESP_LOGI(TAG, "input subscription complete; FD2 notifications armed");
        request_fast_connection_update("input_subscribed");
    }
}

void send_current_init_command() {
    if (s_ble.init_index >= gatt_init_command_count()) {
        mark_stage(Stage::SubscribeInput);
        s_ble.subscribe_index = -1;
        s_ble.pending_subscribe_input = true;
        return;
    }
    if (s_ble.command_handle == 0) {
        ESP_LOGW(TAG, "command characteristic missing; subscribing input anyway");
        mark_stage(Stage::SubscribeInput);
        s_ble.subscribe_index = -1;
        s_ble.pending_subscribe_input = true;
        return;
    }
    const InitCommand &cmd = gatt_init_command(s_ble.init_index);
    const int rc = s_ble.command_no_rsp
        ? ble_gattc_write_no_rsp_flat(s_ble.conn_handle, s_ble.command_handle, cmd.data, cmd.len)
        : ble_gattc_write_flat(s_ble.conn_handle, s_ble.command_handle, cmd.data, cmd.len, write_cb, nullptr);
    ESP_LOGI(TAG, "init %u/%u %s handle=0x%04x rc=%d", s_ble.init_index + 1, gatt_init_command_count(), cmd.name, s_ble.command_handle, rc);
    if (rc == 0) {
        s_ble.init_command_sent_us = esp_timer_get_time();
    }
    if (rc != 0) {
        s_ble.stats.last_error = rc;
        mark_stage(Stage::SubscribeInput);
        s_ble.subscribe_index = -1;
        s_ble.pending_subscribe_input = true;
    }
}

int service_cb(uint16_t conn_handle, const ble_gatt_error *error, const ble_gatt_svc *service, void *arg) {
    (void)conn_handle;
    (void)arg;
    if (error->status == BLE_HS_EDONE) {
        s_ble.discover_service_index = 0;
        discover_next_characteristics();
        return 0;
    }
    if (error->status != 0) {
        s_ble.stats.last_error = error->status;
        ESP_LOGW(TAG, "service discovery error=%d", error->status);
        discover_next_characteristics();
        return 0;
    }
    if (service != nullptr && s_ble.service_count < kMaxServices) {
        s_ble.services[s_ble.service_count++] = ServiceSlot{service->start_handle, service->end_handle};
    }
    return 0;
}

int characteristic_cb(uint16_t conn_handle, const ble_gatt_error *error, const ble_gatt_chr *chr, void *arg) {
    (void)conn_handle;
    (void)arg;
    if (error->status == BLE_HS_EDONE) {
        if (has_minimum_ns2_gatt()) {
            ESP_LOGI(TAG, "minimum NS2 GATT discovered; skipping remaining services");
            mark_stage(Stage::SubscribeAck);
            s_ble.subscribe_index = -1;
            s_ble.pending_subscribe_ack = true;
            return 0;
        }
        s_ble.discover_service_index++;
        discover_next_characteristics();
        return 0;
    }
    if (error->status != 0) {
        s_ble.stats.last_error = error->status;
        s_ble.discover_service_index++;
        discover_next_characteristics();
        return 0;
    }
    if (chr == nullptr || s_ble.char_count >= kMaxChars) {
        return 0;
    }
    CharSlot &slot = s_ble.chars[s_ble.char_count++];
    slot.used = true;
    slot.value_handle = chr->val_handle;
    slot.end_handle = chr->val_handle;
    slot.properties = chr->properties;
    uuid_to_be(&chr->uuid.u, slot.uuid_be);
    slot.role = gatt_classify_uuid(slot.uuid_be);
    if (slot.role == GattRole::Command) {
        s_ble.command_handle = slot.value_handle;
        s_ble.command_no_rsp = (slot.properties & BLE_GATT_CHR_F_WRITE_NO_RSP) != 0;
    } else if (slot.role == GattRole::Rumble) {
        s_ble.rumble_handle = slot.value_handle;
        s_ble.rumble_no_rsp = (slot.properties & BLE_GATT_CHR_F_WRITE_NO_RSP) != 0;
    }
    char uuid[48];
    gatt_format_uuid(slot.uuid_be, uuid, sizeof(uuid));
    ESP_LOGI(TAG, "char value=0x%04x props=0x%02x role=%s uuid=%s", slot.value_handle, slot.properties, gatt_role_name(slot.role), uuid);
    return 0;
}

int write_cb(uint16_t conn_handle, const ble_gatt_error *error, ble_gatt_attr *attr, void *arg) {
    (void)conn_handle;
    (void)attr;
    const GattRole role = static_cast<GattRole>(reinterpret_cast<uintptr_t>(arg));
    if (error != nullptr && error->status != 0) {
        s_ble.stats.last_error = error->status;
        ESP_LOGW(TAG, "write complete role=%s status=%d", gatt_role_name(role), error->status);
    }
    if (s_ble.stage == Stage::SubscribeAck) {
        if (s_ble.subscribe_index >= 0 && s_ble.subscribe_index < static_cast<int>(s_ble.char_count)) {
            s_ble.chars[s_ble.subscribe_index].subscribed = true;
        }
        s_ble.pending_subscribe_ack = true;
    } else if (s_ble.stage == Stage::SubscribeInput) {
        if (s_ble.subscribe_index >= 0 && s_ble.subscribe_index < static_cast<int>(s_ble.char_count)) {
            s_ble.chars[s_ble.subscribe_index].subscribed = true;
        }
        s_ble.pending_subscribe_input = true;
    } else if (s_ble.stage == Stage::Initializing && !s_ble.command_no_rsp) {
        s_ble.init_index++;
        send_current_init_command();
    }
    update_ready_flags();
    return 0;
}

int mtu_cb(uint16_t conn_handle, const ble_gatt_error *error, uint16_t mtu, void *arg) {
    (void)conn_handle;
    (void)arg;
    ESP_LOGI(TAG, "MTU exchange status=%d mtu=%u", error ? error->status : 0, mtu);
    start_service_discovery();
    return 0;
}

void handle_notification(uint16_t attr_handle, os_mbuf *om) {
    uint8_t data[128];
    const uint16_t len = std::min<uint16_t>(OS_MBUF_PKTLEN(om), sizeof(data));
    os_mbuf_copydata(om, 0, len, data);
    CharSlot *slot = find_char_by_handle(attr_handle);
    if (slot == nullptr) {
        ESP_LOGW(TAG, "notify unknown handle=0x%04x len=%u", attr_handle, len);
        return;
    }
    if (slot->role == GattRole::AckNotify && s_ble.stage == Stage::Initializing) {
        if (s_ble.ack_notify_logged < 20) {
            ESP_LOGI(TAG, "ack notify handle=0x%04x len=%u init_index=%u", attr_handle, len, s_ble.init_index);
            s_ble.ack_notify_logged++;
        }
        s_ble.init_index++;
        send_current_init_command();
        return;
    }
    if (slot->role == GattRole::InputNotify) {
        const InputReportKind kind = gatt_uuid128_equals(slot->uuid_be, gatt_fd2_uuid())
            ? InputReportKind::Fd2
            : (gatt_uuid128_equals(slot->uuid_be, gatt_legacy_uuid()) ? InputReportKind::Legacy : InputReportKind::Unknown);
        if (s_ble.input_notify_logged < 20) {
            ESP_LOGI(TAG, "input notify handle=0x%04x len=%u kind=%u", attr_handle, len, static_cast<unsigned>(kind));
            s_ble.input_notify_logged++;
        }
        if (input_parse_notify(kind, data, len)) {
            note_input_notify_rate();
            mark_stage(Stage::Ready);
            if (!s_ble.target_saved_this_connection) {
                config_set_saved_target(s_ble.pending_addr.val, s_ble.pending_addr.type);
                config_save();
                s_ble.target_saved_this_connection = true;
                ESP_LOGI(TAG, "saved live target addr_type=%u", s_ble.pending_addr.type);
            }
        } else {
            s_ble.stats.last_error = -1000;
            if (s_ble.input_parse_failed_logged < 20) {
                ESP_LOGW(TAG,
                         "input parse rejected handle=0x%04x len=%u kind=%u head=%02x %02x %02x %02x %02x %02x %02x %02x",
                         attr_handle,
                         len,
                         static_cast<unsigned>(kind),
                         len > 0 ? data[0] : 0,
                         len > 1 ? data[1] : 0,
                         len > 2 ? data[2] : 0,
                         len > 3 ? data[3] : 0,
                         len > 4 ? data[4] : 0,
                         len > 5 ? data[5] : 0,
                         len > 6 ? data[6] : 0,
                         len > 7 ? data[7] : 0);
                s_ble.input_parse_failed_logged++;
            }
        }
    }
}

void connect_to_candidate(const ble_gap_disc_desc &disc) {
    const uint64_t now = esp_timer_get_time();
    if (s_ble.next_connect_allowed_us != 0 && now < s_ble.next_connect_allowed_us) {
        return;
    }
    ble_gap_disc_cancel();
    s_ble.pending_addr = disc.addr;
    mark_stage(Stage::Connecting);
    s_ble.stats.scanning = false;
    s_ble.stats.connect_attempts++;
    s_ble.connect_started_us = now;
    s_ble.next_connect_allowed_us = now + kConnectRetryBaseDelayUs;
    const ble_gap_conn_params params = {
        .scan_itvl = 0x0010,
        .scan_window = 0x0010,
        .itvl_min = kFastConnIntervalMin,
        .itvl_max = kFastConnIntervalMax,
        .latency = kFastConnLatency,
        .supervision_timeout = kFastConnSupervisionTimeout,
        .min_ce_len = 0,
        .max_ce_len = 0,
    };
    const int rc = ble_gap_connect(s_ble.own_addr_type, &disc.addr, kConnectTimeoutMs, &params, gap_event, nullptr);
    if (rc != 0) {
        s_ble.stats.last_error = rc;
        ESP_LOGE(TAG, "connect start failed rc=%d", rc);
        s_ble.connect_started_us = 0;
        s_ble.consecutive_connect_failures++;
        schedule_scan(connect_retry_delay_us());
    }
}

AdvMatch match_adv_fields(const ble_hs_adv_fields &fields) {
    AdvMatch match{};
    match.name = name_looks_like_controller(fields.name, fields.name_len);
    match.manufacturer = manufacturer_looks_pairable(fields.mfg_data, fields.mfg_data_len);
    for (uint8_t i = 0; i < fields.num_uuids128; ++i) {
        if (uuid128_matches_be_uuid(fields.uuids128[i].value, fd2_notify_uuid_be())) {
            match.service_uuid = true;
            break;
        }
    }
    return match;
}

void log_candidate(const ble_gap_disc_desc &disc, const ble_hs_adv_fields &fields, const AdvMatch &match) {
    char addr[18] = {};
    format_ble_addr(disc.addr.val, addr, sizeof(addr));
    char name[32] = {};
    const size_t name_len = std::min<size_t>(fields.name_len, sizeof(name) - 1);
    if (fields.name != nullptr && name_len > 0) {
        std::memcpy(name, fields.name, name_len);
    }
    s_ble.stats.candidates++;
    s_ble.stats.last_rssi = disc.rssi;
    std::memcpy(s_ble.stats.last_addr, addr, sizeof(s_ble.stats.last_addr));
    std::memcpy(s_ble.stats.last_name, name, sizeof(s_ble.stats.last_name));
    ESP_LOGI(TAG, "candidate addr=%s type=%u rssi=%d name=\"%s\" match(name=%d mfg=%d uuid=%d)",
             addr, disc.addr.type, disc.rssi, name, match.name, match.manufacturer, match.service_uuid);
}

void format_hex(const uint8_t *data, size_t len, char *out, size_t out_len) {
    if (out == nullptr || out_len == 0) {
        return;
    }
    out[0] = 0;
    if (data == nullptr) {
        return;
    }
    size_t used = 0;
    for (size_t i = 0; i < len && used + 2 < out_len; ++i) {
        const int written = snprintf(out + used, out_len - used, "%02x", data[i]);
        if (written <= 0) {
            break;
        }
        used += static_cast<size_t>(written);
    }
}

void format_uuid_le_as_be(const uint8_t uuid_le[kUuid128Len], char *out, size_t out_len) {
    if (out == nullptr || out_len == 0) {
        return;
    }
    if (uuid_le == nullptr) {
        snprintf(out, out_len, "<none>");
        return;
    }
    uint8_t uuid_be[kUuid128Len] = {};
    for (size_t i = 0; i < kUuid128Len; ++i) {
        uuid_be[i] = uuid_le[kUuid128Len - 1 - i];
    }
    gatt_format_uuid(uuid_be, out, out_len);
}

void log_adv_probe(const ble_gap_disc_desc &disc, const ble_hs_adv_fields &fields, const AdvMatch &match) {
    s_ble.stats.adv_seen++;
    if (s_ble.stats.adv_logged >= 80) {
        return;
    }
    const bool interesting = match.likely_controller() ||
                             fields.name_len > 0 ||
                             fields.mfg_data_len > 0 ||
                             fields.num_uuids128 > 0;
    if (!interesting) {
        return;
    }

    char addr[18] = {};
    char name[32] = {};
    char mfg[96] = {};
    char uuid0[48] = {};
    format_ble_addr(disc.addr.val, addr, sizeof(addr));
    const size_t name_len = std::min<size_t>(fields.name_len, sizeof(name) - 1);
    if (fields.name != nullptr && name_len > 0) {
        std::memcpy(name, fields.name, name_len);
    }
    format_hex(fields.mfg_data, std::min<size_t>(fields.mfg_data_len, 40), mfg, sizeof(mfg));
    if (fields.num_uuids128 > 0) {
        format_uuid_le_as_be(fields.uuids128[0].value, uuid0, sizeof(uuid0));
    }
    ESP_LOGI(TAG,
             "adv probe addr=%s type=%u rssi=%d name=\"%s\" mfg=%s uuid128=%u uuid0=%s match=%d/%d/%d",
             addr,
             disc.addr.type,
             disc.rssi,
             name,
             mfg,
             fields.num_uuids128,
             uuid0,
             match.name,
             match.manufacturer,
             match.service_uuid);
    s_ble.stats.adv_logged++;
}

void start_scan() {
    if (s_ble.conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        return;
    }
    const uint64_t now = esp_timer_get_time();
    if (s_ble.scan_restart_due_us != 0 && now < s_ble.scan_restart_due_us) {
        return;
    }
    s_ble.scan_restart_due_us = 0;
    s_ble.stay_disconnected = false;
    if (!s_ble.manual_scan && !s_ble.pair_mode && !config_auto_connect()) {
        s_ble.stats.scanning = false;
        mark_stage(Stage::Idle);
        ESP_LOGI(TAG, "auto scan skipped; auto_connect disabled");
        return;
    }
    stop_scan();
    ble_gap_disc_params params{};
    params.filter_duplicates = 1;
    params.passive = 0;
    params.itvl = 0x0010;
    params.window = 0x0010;
    const int rc = ble_gap_disc(s_ble.own_addr_type, BLE_HS_FOREVER, &params, gap_event, nullptr);
    if (rc != 0) {
        s_ble.stats.scanning = false;
        s_ble.stats.scan_errors++;
        s_ble.stats.last_error = rc;
        ESP_LOGE(TAG, "ble_gap_disc failed rc=%d", rc);
    } else {
        mark_stage(Stage::Scanning);
        s_ble.stats.scanning = true;
        s_ble.stats.scan_starts++;
        ESP_LOGI(TAG, "BLE scan started");
    }
}

void schedule_scan(uint64_t delay_us) {
    if (s_ble.stay_disconnected) {
        return;
    }
    stop_scan();
    s_ble.scan_restart_due_us = esp_timer_get_time() + delay_us;
    mark_stage(Stage::Idle);
    ESP_LOGI(TAG, "BLE scan scheduled in %llu ms", delay_us / 1000ULL);
}

void stop_scan() {
    if (s_ble.stats.scanning) {
        ble_gap_disc_cancel();
        s_ble.stats.scanning = false;
    }
}

bool addr_equals(const uint8_t a[6], const uint8_t b[6]) {
    return std::memcmp(a, b, 6) == 0;
}

bool disc_matches_saved(const ble_gap_disc_desc &disc) {
    if (!config_has_saved_target()) {
        return false;
    }
    uint8_t saved[6];
    uint8_t type = 0;
    config_get_saved_target(saved, &type);
    return disc.addr.type == type && addr_equals(disc.addr.val, saved);
}

void disconnect_current(const char *reason) {
    ESP_LOGW(TAG, "disconnect_current reason=%s", reason ? reason : "manual");
    if (s_ble.conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        ble_gap_terminate(s_ble.conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    } else {
        stop_scan();
        clear_gatt();
        input_reset();
        start_scan();
    }
}

int gap_event(ble_gap_event *event, void *arg) {
    (void)arg;
    switch (event->type) {
    case BLE_GAP_EVENT_DISC: {
        ble_hs_adv_fields fields{};
        if (ble_hs_adv_parse_fields(&fields, event->disc.data, event->disc.length_data) != 0) {
            return 0;
        }
        const AdvMatch match = match_adv_fields(fields);
        log_adv_probe(event->disc, fields, match);
        const bool saved_match = disc_matches_saved(event->disc);
        const bool candidate = match.likely_controller();
        if (candidate || saved_match) {
            log_candidate(event->disc, fields, match);
            if (saved_match || event->disc.rssi >= kConnectRssiMin) {
                connect_to_candidate(event->disc);
            }
        }
        return 0;
    }
    case BLE_GAP_EVENT_DISC_COMPLETE:
        s_ble.stats.scanning = false;
        start_scan();
        return 0;
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            s_ble.conn_handle = event->connect.conn_handle;
            s_ble.connect_started_us = 0;
            s_ble.next_connect_allowed_us = 0;
            s_ble.scan_restart_due_us = 0;
            s_ble.consecutive_connect_failures = 0;
            s_ble.stats.connected = true;
            s_ble.stats.scanning = false;
            update_conn_desc();
            request_fast_link_params("connect");
            request_fast_connection_update("connect");
            ESP_LOGI(TAG, "connected handle=%u", s_ble.conn_handle);
            ble_gattc_exchange_mtu(s_ble.conn_handle, mtu_cb, nullptr);
        } else {
            s_ble.stats.last_error = event->connect.status;
            s_ble.conn_handle = BLE_HS_CONN_HANDLE_NONE;
            s_ble.connect_started_us = 0;
            s_ble.consecutive_connect_failures++;
            s_ble.next_connect_allowed_us = esp_timer_get_time() + connect_retry_delay_us();
            ESP_LOGW(TAG, "connect failed status=%d", event->connect.status);
            schedule_scan(connect_retry_delay_us());
        }
        return 0;
    case BLE_GAP_EVENT_DISCONNECT: {
        ESP_LOGI(TAG, "disconnect reason=%d", event->disconnect.reason);
        const bool had_connection = s_ble.conn_handle != BLE_HS_CONN_HANDLE_NONE || s_ble.stats.connected;
        s_ble.stats.disconnect_count++;
        s_ble.stats.last_disconnect_reason = event->disconnect.reason;
        s_ble.conn_handle = BLE_HS_CONN_HANDLE_NONE;
        s_ble.connect_started_us = 0;
        clear_gatt();
        input_reset();
        s_ble.pair_mode = false;
        s_ble.manual_scan = false;
        if (s_ble.stay_disconnected) {
            s_ble.stay_disconnected = false;
            mark_stage(Stage::Idle);
        } else {
            if (had_connection) {
                s_ble.consecutive_connect_failures = 0;
                schedule_scan(kReconnectAfterReadyDelayUs);
            } else {
                s_ble.consecutive_connect_failures++;
                s_ble.next_connect_allowed_us = esp_timer_get_time() + connect_retry_delay_us();
                schedule_scan(connect_retry_delay_us());
            }
        }
        return 0;
    }
    case BLE_GAP_EVENT_NOTIFY_RX:
        handle_notification(event->notify_rx.attr_handle, event->notify_rx.om);
        update_ready_flags();
        return 0;
    case BLE_GAP_EVENT_CONN_UPDATE:
        s_ble.stats.conn_update_status = event->conn_update.status;
        update_conn_desc();
        ESP_LOGI(TAG,
                 "conn update status=%d interval=%u (%lu us) latency=%u supervision=%u",
                 event->conn_update.status,
                 static_cast<unsigned>(s_ble.stats.conn_interval_units),
                 static_cast<unsigned long>(s_ble.stats.conn_interval_us),
                 static_cast<unsigned>(s_ble.stats.conn_latency),
                 static_cast<unsigned>(s_ble.stats.conn_supervision_timeout));
        return 0;
    case BLE_GAP_EVENT_CONN_UPDATE_REQ: {
        ble_gap_upd_params params{};
        params.itvl_min = kFastConnIntervalMin;
        params.itvl_max = kFastConnIntervalMax;
        params.latency = kFastConnLatency;
        params.supervision_timeout = kFastConnSupervisionTimeout;
        params.min_ce_len = 0;
        params.max_ce_len = 0;
        ESP_LOGI(TAG,
                 "conn update req peer=%u..%u latency=%u supervision=%u; reply fast=%u..%u",
                 event->conn_update_req.peer_params ? event->conn_update_req.peer_params->itvl_min : 0,
                 event->conn_update_req.peer_params ? event->conn_update_req.peer_params->itvl_max : 0,
                 event->conn_update_req.peer_params ? event->conn_update_req.peer_params->latency : 0,
                 event->conn_update_req.peer_params ? event->conn_update_req.peer_params->supervision_timeout : 0,
                 static_cast<unsigned>(params.itvl_min),
                 static_cast<unsigned>(params.itvl_max));
        if (event->conn_update_req.self_params != nullptr) {
            *event->conn_update_req.self_params = params;
        }
        return 0;
    }
    case BLE_GAP_EVENT_PHY_UPDATE_COMPLETE:
        s_ble.stats.phy_update_status = event->phy_updated.status;
        update_conn_desc();
        ESP_LOGI(TAG,
                 "phy update status=%d tx=%u rx=%u",
                 event->phy_updated.status,
                 static_cast<unsigned>(s_ble.stats.tx_phy),
                 static_cast<unsigned>(s_ble.stats.rx_phy));
        return 0;
    case BLE_GAP_EVENT_DATA_LEN_CHG:
        s_ble.stats.data_len_tx_octets = event->data_len_chg.max_tx_octets;
        s_ble.stats.data_len_rx_octets = event->data_len_chg.max_rx_octets;
        s_ble.stats.data_len_tx_time = event->data_len_chg.max_tx_time;
        s_ble.stats.data_len_rx_time = event->data_len_chg.max_rx_time;
        ESP_LOGI(TAG,
                 "data len tx=%u/%u rx=%u/%u",
                 static_cast<unsigned>(s_ble.stats.data_len_tx_octets),
                 static_cast<unsigned>(s_ble.stats.data_len_tx_time),
                 static_cast<unsigned>(s_ble.stats.data_len_rx_octets),
                 static_cast<unsigned>(s_ble.stats.data_len_rx_time));
        return 0;
    default:
        return 0;
    }
}

void on_reset(int reason) {
    s_ble.stats.last_error = reason;
    ESP_LOGE(TAG, "NimBLE reset reason=%d", reason);
}

void on_sync() {
    int rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        s_ble.stats.last_error = rc;
        return;
    }
    rc = ble_hs_id_infer_auto(0, &s_ble.own_addr_type);
    if (rc != 0) {
        s_ble.stats.last_error = rc;
        return;
    }
    uint8_t local_addr[6] = {};
    if (ble_hs_id_copy_addr(s_ble.own_addr_type, local_addr, nullptr) == 0) {
        std::memcpy(s_ble.stats.local_addr, local_addr, sizeof(s_ble.stats.local_addr));
        s_ble.stats.local_addr_valid = true;
        gatt_set_console_mac(local_addr);
    }
    s_ble.stats.own_addr_type = s_ble.own_addr_type;
    if (config_auto_connect()) {
        start_scan();
    }
}

void host_task(void *param) {
    (void)param;
    ESP_LOGI(TAG, "NimBLE host task started");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

esp_err_t init_nvs_for_ble() {
    return config_init();
}

} // namespace

esp_err_t ble_start() {
    if (s_ble.started) {
        return ESP_OK;
    }
    s_ble.conn_handle = BLE_HS_CONN_HANDLE_NONE;
    input_reset();

    ESP_RETURN_ON_ERROR(init_nvs_for_ble(), TAG, "NVS init failed");
    ESP_RETURN_ON_ERROR(nimble_port_init(), TAG, "nimble_port_init failed");

    ble_hs_cfg.reset_cb = on_reset;
    ble_hs_cfg.sync_cb = on_sync;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
    ble_hs_cfg.sm_io_cap = BLE_SM_IO_CAP_NO_IO;
    ble_hs_cfg.sm_bonding = 1;
    ble_hs_cfg.sm_mitm = 0;
    ble_hs_cfg.sm_sc = 1;
    ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;

    ble_svc_gap_init();
    ble_svc_gap_device_name_set("ns2pro-esp32s3");
    ble_store_config_init();
    nimble_port_freertos_init(host_task);

    s_ble.started = true;
    s_ble.stats.started = true;
    ESP_LOGI(TAG, "NimBLE central connector initialized");
    return ESP_OK;
}

void ble_task() {
    const uint64_t now = esp_timer_get_time();
    if (s_ble.conn_handle == BLE_HS_CONN_HANDLE_NONE &&
        s_ble.stage == Stage::Connecting &&
        s_ble.connect_started_us != 0 &&
        now - s_ble.connect_started_us > kConnectTimeoutUs) {
        ESP_LOGW(TAG, "connect timeout; restarting scan");
        ble_gap_conn_cancel();
        s_ble.connect_started_us = 0;
        s_ble.consecutive_connect_failures++;
        s_ble.next_connect_allowed_us = now + connect_retry_delay_us();
        clear_gatt();
        schedule_scan(connect_retry_delay_us());
        return;
    }
    if (s_ble.conn_handle == BLE_HS_CONN_HANDLE_NONE &&
        s_ble.scan_restart_due_us != 0 &&
        now >= s_ble.scan_restart_due_us) {
        s_ble.scan_restart_due_us = 0;
        start_scan();
        return;
    }
    if (s_ble.conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        return;
    }
    if (s_ble.stage == Stage::Initializing &&
        s_ble.init_command_sent_us != 0 &&
        now - s_ble.init_command_sent_us > kInitAckTimeoutUs) {
        disconnect_current("init ACK timeout");
        return;
    }
    if (s_ble.stage == Stage::Ready) {
        const uint32_t age = input_last_age_ms();
        if (age != 0xffffffffu && age > kNotifyStaleLogUs / 1000ULL &&
            (s_ble.next_notify_stale_log_us == 0 || now >= s_ble.next_notify_stale_log_us)) {
            ESP_LOGW(TAG, "input notify stale age_ms=%lu; keeping BLE link",
                     static_cast<unsigned long>(age));
            s_ble.next_notify_stale_log_us = now + kNotifyStaleLogUs;
        }
    }
    if (s_ble.next_gatt_action_us != 0 && now < s_ble.next_gatt_action_us) {
        return;
    }
    if (s_ble.pending_subscribe_ack && s_ble.stage == Stage::SubscribeAck) {
        s_ble.pending_subscribe_ack = false;
        subscribe_role(GattRole::AckNotify);
        s_ble.next_gatt_action_us = now + kCommandSpacingUs;
        return;
    }
    if (s_ble.pending_subscribe_input && s_ble.stage == Stage::SubscribeInput) {
        s_ble.pending_subscribe_input = false;
        subscribe_role(GattRole::InputNotify);
        s_ble.next_gatt_action_us = now + kCommandSpacingUs;
        return;
    }
}

void ble_get_stats(BleStats *out) {
    if (out == nullptr) {
        return;
    }
    update_ready_flags();
    update_conn_desc();
    *out = s_ble.stats;
    out->auto_connect = config_auto_connect();
    out->saved_target_valid = config_has_saved_target();
    out->pair_mode = s_ble.pair_mode;
    if (out->saved_target_valid) {
        config_get_saved_target(out->saved_addr, &out->saved_addr_type);
    }
    if (s_ble.conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        out->scanning = false;
    }
}

bool ble_send_rumble(const uint8_t *data, uint16_t len) {
    if (data == nullptr || len == 0 || s_ble.conn_handle == BLE_HS_CONN_HANDLE_NONE || s_ble.rumble_handle == 0) {
        s_ble.stats.rumble_errors++;
        return false;
    }
    const int rc = s_ble.rumble_no_rsp
        ? ble_gattc_write_no_rsp_flat(s_ble.conn_handle, s_ble.rumble_handle, data, len)
        : ble_gattc_write_flat(s_ble.conn_handle, s_ble.rumble_handle, data, len, write_cb, nullptr);
    if (rc == 0) {
        s_ble.stats.rumble_writes++;
        return true;
    }
    s_ble.stats.rumble_errors++;
    s_ble.stats.last_error = rc;
    return false;
}

void ble_start_scan() {
    s_ble.manual_scan = true;
    s_ble.pair_mode = false;
    s_ble.next_connect_allowed_us = 0;
    s_ble.scan_restart_due_us = 0;
    if (s_ble.conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        disconnect_current("scan requested");
        return;
    }
    start_scan();
}

void ble_pair() {
    s_ble.manual_scan = true;
    s_ble.pair_mode = true;
    s_ble.next_connect_allowed_us = 0;
    s_ble.scan_restart_due_us = 0;
    if (s_ble.conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        disconnect_current("pair requested");
        return;
    }
    start_scan();
}

void ble_reconnect() {
    s_ble.manual_scan = false;
    s_ble.pair_mode = false;
    s_ble.next_connect_allowed_us = 0;
    s_ble.scan_restart_due_us = 0;
    if (s_ble.conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        disconnect_current("reconnect requested");
        return;
    }
    start_scan();
}

void ble_disconnect() {
    stop_scan();
    s_ble.manual_scan = false;
    s_ble.pair_mode = false;
    s_ble.stay_disconnected = true;
    s_ble.scan_restart_due_us = 0;
    s_ble.next_connect_allowed_us = 0;
    if (s_ble.conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        ble_gap_terminate(s_ble.conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        return;
    }
    mark_stage(Stage::Idle);
}

void ble_forget() {
    config_forget_saved_target();
    config_save();
    ble_disconnect();
}

void ble_set_auto_connect(bool enabled) {
    config_set_auto_connect(enabled);
    config_save();
    if (enabled && s_ble.conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        start_scan();
    } else if (!enabled && s_ble.conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        stop_scan();
        mark_stage(Stage::Idle);
    }
}

} // namespace ns2
