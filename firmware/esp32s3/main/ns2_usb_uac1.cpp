#include "ns2_usb.h"

#include <cstdint>
#include <cstring>

#include "class/audio/audio.h"
#include "device/usbd_pvt.h"
#include "esp_log.h"
#include "tusb.h"

namespace {

constexpr uint8_t kAudioControlInterface = 0;
constexpr uint8_t kAudioOutInterface = 1;
constexpr uint8_t kAudioInInterface = 2;
constexpr uint8_t kAudioOutEp = 0x01;
constexpr uint8_t kAudioInEp = 0x82;
constexpr uint8_t kSpeakerFeatureUnit = 0x02;
constexpr uint8_t kMicFeatureUnit = 0x05;
constexpr uint8_t kAltIdle = 0;
constexpr uint8_t kAltStreaming = 1;
constexpr uint16_t kAudioOutPacketBytes = 384;
constexpr uint16_t kAudioControlDescriptorLen = 82;
constexpr uint16_t kAudioOutDescriptorLen = 52;
constexpr uint16_t kAudioInDescriptorLen = 52;

constexpr uint8_t kSetCur = 0x01;
constexpr uint8_t kGetCur = 0x81;
constexpr uint8_t kGetMin = 0x82;
constexpr uint8_t kGetMax = 0x83;
constexpr uint8_t kGetRes = 0x84;

const char *TAG = "ns2-uac1";

uint8_t s_out_alt = kAltIdle;
uint8_t s_in_alt = kAltIdle;
uint8_t s_mute[2] = {};
int16_t s_volume[2] = {-100 * 256, 0};
uint8_t s_control_buffer[2] = {};
uint32_t s_packet_count = 0;
uint8_t s_audio_out_buffer[kAudioOutPacketBytes] CFG_TUSB_MEM_SECTION TU_ATTR_ALIGNED(4) = {};

const tusb_desc_endpoint_t s_audio_out_endpoint = {
    .bLength = sizeof(tusb_desc_endpoint_t),
    .bDescriptorType = TUSB_DESC_ENDPOINT,
    .bEndpointAddress = kAudioOutEp,
    .bmAttributes = {
        .xfer = TUSB_XFER_ISOCHRONOUS,
        .sync = (TUSB_ISO_EP_ATT_ADAPTIVE >> 2),
        .usage = (TUSB_ISO_EP_ATT_DATA >> 4),
    },
    .wMaxPacketSize = kAudioOutPacketBytes,
    .bInterval = 1,
};

int feature_index(uint8_t entity_id) {
    if (entity_id == kSpeakerFeatureUnit) {
        return 0;
    }
    if (entity_id == kMicFeatureUnit) {
        return 1;
    }
    return -1;
}

void write_le16(uint8_t *out, int16_t value) {
    out[0] = static_cast<uint8_t>(value & 0xff);
    out[1] = static_cast<uint8_t>((static_cast<uint16_t>(value) >> 8) & 0xff);
}

void uac1_init() {
    s_out_alt = kAltIdle;
    s_in_alt = kAltIdle;
    s_packet_count = 0;
    std::memset(s_mute, 0, sizeof(s_mute));
    s_volume[0] = -100 * 256;
    s_volume[1] = 0;
    std::memset(s_control_buffer, 0, sizeof(s_control_buffer));
    std::memset(s_audio_out_buffer, 0, sizeof(s_audio_out_buffer));
}

bool uac1_deinit() {
    return true;
}

void uac1_reset(uint8_t rhport) {
    (void)rhport;
    s_out_alt = kAltIdle;
    s_in_alt = kAltIdle;
    s_packet_count = 0;
}

uint16_t uac1_open(uint8_t rhport,
                   tusb_desc_interface_t const *interface_descriptor,
                   uint16_t max_len) {
    if (interface_descriptor->bInterfaceClass != TUSB_CLASS_AUDIO ||
        interface_descriptor->bInterfaceProtocol != AUDIO_INT_PROTOCOL_CODE_UNDEF) {
        return 0;
    }

    if (interface_descriptor->bInterfaceNumber == kAudioControlInterface &&
        interface_descriptor->bInterfaceSubClass == AUDIO_SUBCLASS_CONTROL &&
        interface_descriptor->bAlternateSetting == 0) {
        if (max_len < kAudioControlDescriptorLen) {
            return 0;
        }
        ESP_LOGI(TAG, "open audio control len=%u", kAudioControlDescriptorLen);
        return kAudioControlDescriptorLen;
    }

    if (interface_descriptor->bInterfaceNumber == kAudioOutInterface &&
        interface_descriptor->bInterfaceSubClass == AUDIO_SUBCLASS_STREAMING &&
        interface_descriptor->bAlternateSetting == 0) {
        if (max_len < kAudioOutDescriptorLen ||
            !usbd_edpt_iso_alloc(rhport, kAudioOutEp, kAudioOutPacketBytes)) {
            ESP_LOGE(TAG, "audio OUT endpoint allocation failed");
            return 0;
        }
        ESP_LOGI(TAG, "open audio OUT len=%u packet=%u", kAudioOutDescriptorLen, kAudioOutPacketBytes);
        return kAudioOutDescriptorLen;
    }

    if (interface_descriptor->bInterfaceNumber == kAudioInInterface &&
        interface_descriptor->bInterfaceSubClass == AUDIO_SUBCLASS_STREAMING &&
        interface_descriptor->bAlternateSetting == 0) {
        if (max_len < kAudioInDescriptorLen) {
            return 0;
        }
        ESP_LOGI(TAG, "open microphone compatibility interface len=%u", kAudioInDescriptorLen);
        return kAudioInDescriptorLen;
    }

    return 0;
}

bool start_audio_out(uint8_t rhport) {
    if (!usbd_edpt_iso_activate(rhport, &s_audio_out_endpoint)) {
        ESP_LOGE(TAG, "audio OUT endpoint activation failed");
        return false;
    }
    s_packet_count = 0;
    const bool armed = usbd_edpt_xfer(
        rhport, kAudioOutEp, s_audio_out_buffer, sizeof(s_audio_out_buffer));
    ESP_LOGI(TAG, "audio OUT streaming=%s", armed ? "true" : "false");
    return armed;
}

bool feature_control_setup(uint8_t rhport, tusb_control_request_t const *request) {
    const uint8_t interface_number = TU_U16_LOW(request->wIndex);
    const uint8_t entity_id = TU_U16_HIGH(request->wIndex);
    const uint8_t channel = TU_U16_LOW(request->wValue);
    const uint8_t selector = TU_U16_HIGH(request->wValue);
    const int index = feature_index(entity_id);

    if (interface_number != kAudioControlInterface || index < 0 || channel != 0) {
        return false;
    }

    if (request->bmRequestType_bit.direction == TUSB_DIR_OUT) {
        if (request->bRequest != kSetCur) {
            return false;
        }
        const uint16_t expected_len = selector == AUDIO_FU_CTRL_MUTE ? 1 :
                                      selector == AUDIO_FU_CTRL_VOLUME ? 2 : 0;
        return expected_len != 0 && request->wLength == expected_len &&
               tud_control_xfer(rhport, request, s_control_buffer, expected_len);
    }

    if (selector == AUDIO_FU_CTRL_MUTE) {
        if (request->bRequest != kGetCur || request->wLength != 1) {
            return false;
        }
        s_control_buffer[0] = s_mute[index];
        return tud_control_xfer(rhport, request, s_control_buffer, 1);
    }

    if (selector != AUDIO_FU_CTRL_VOLUME || request->wLength != 2) {
        return false;
    }

    int16_t value = 0;
    switch (request->bRequest) {
    case kGetCur:
        value = s_volume[index];
        break;
    case kGetMin:
        value = entity_id == kSpeakerFeatureUnit ? static_cast<int16_t>(0x9c00) : 0x0000;
        break;
    case kGetMax:
        value = entity_id == kSpeakerFeatureUnit ? 0x0000 : 0x3000;
        break;
    case kGetRes:
        value = entity_id == kSpeakerFeatureUnit ? 0x0100 : 0x007a;
        break;
    default:
        return false;
    }
    write_le16(s_control_buffer, value);
    return tud_control_xfer(rhport, request, s_control_buffer, 2);
}

bool feature_control_data(tusb_control_request_t const *request) {
    const uint8_t entity_id = TU_U16_HIGH(request->wIndex);
    const uint8_t selector = TU_U16_HIGH(request->wValue);
    const int index = feature_index(entity_id);
    if (index < 0 || request->bRequest != kSetCur) {
        return false;
    }
    if (selector == AUDIO_FU_CTRL_MUTE && request->wLength == 1) {
        s_mute[index] = s_control_buffer[0] ? 1 : 0;
        return true;
    }
    if (selector == AUDIO_FU_CTRL_VOLUME && request->wLength == 2) {
        s_volume[index] = static_cast<int16_t>(
            static_cast<uint16_t>(s_control_buffer[0]) |
            (static_cast<uint16_t>(s_control_buffer[1]) << 8));
        return true;
    }
    return false;
}

bool uac1_control_xfer(uint8_t rhport,
                       uint8_t stage,
                       tusb_control_request_t const *request) {
    if (stage == CONTROL_STAGE_DATA) {
        if (request->bmRequestType_bit.type == TUSB_REQ_TYPE_CLASS &&
            request->bmRequestType_bit.recipient == TUSB_REQ_RCPT_INTERFACE &&
            request->bmRequestType_bit.direction == TUSB_DIR_OUT) {
            return feature_control_data(request);
        }
        return true;
    }
    if (stage == CONTROL_STAGE_ACK) {
        return true;
    }
    if (stage != CONTROL_STAGE_SETUP ||
        request->bmRequestType_bit.recipient != TUSB_REQ_RCPT_INTERFACE) {
        return false;
    }
    if (request->bmRequestType_bit.type == TUSB_REQ_TYPE_CLASS) {
        return feature_control_setup(rhport, request);
    }
    if (request->bmRequestType_bit.type != TUSB_REQ_TYPE_STANDARD) {
        return false;
    }

    const uint8_t interface_number = TU_U16_LOW(request->wIndex);
    const bool is_out = interface_number == kAudioOutInterface;
    const bool is_in = interface_number == kAudioInInterface;
    if (!is_out && !is_in) {
        return false;
    }

    uint8_t *alt_setting = is_out ? &s_out_alt : &s_in_alt;
    if (request->bRequest == TUSB_REQ_GET_INTERFACE) {
        return tud_control_xfer(rhport, request, alt_setting, sizeof(*alt_setting));
    }
    if (request->bRequest != TUSB_REQ_SET_INTERFACE) {
        return false;
    }

    const uint8_t alt = TU_U16_LOW(request->wValue);
    if (alt > kAltStreaming) {
        return false;
    }
    if (is_in && alt == kAltStreaming) {
        ESP_LOGW(TAG, "microphone streaming rejected; interface is descriptor-only");
        return false;
    }
    if (is_out && alt == kAltStreaming && s_out_alt != kAltStreaming && !start_audio_out(rhport)) {
        return false;
    }

    *alt_setting = alt;
    ns2::usb_note_uac1_interface(interface_number, alt);
    ESP_LOGI(TAG, "set interface=%u alt=%u", interface_number, alt);
    return tud_control_status(rhport, request);
}

bool uac1_xfer(uint8_t rhport,
               uint8_t ep_addr,
               xfer_result_t result,
               uint32_t xferred_bytes) {
    if (ep_addr != kAudioOutEp) {
        return ep_addr == kAudioInEp;
    }

    if (result == XFER_RESULT_SUCCESS && xferred_bytes > 0 &&
        xferred_bytes <= sizeof(s_audio_out_buffer)) {
        ns2::usb_submit_dualsense_audio_packet(
            s_audio_out_buffer, static_cast<uint16_t>(xferred_bytes));
        s_packet_count++;
        if (s_packet_count == 1 || (s_packet_count % 5000) == 0) {
            ESP_LOGI(TAG, "audio OUT packet len=%lu count=%lu",
                     static_cast<unsigned long>(xferred_bytes),
                     static_cast<unsigned long>(s_packet_count));
        }
    }

    if (s_out_alt != kAltStreaming) {
        return true;
    }
    const bool rearmed = usbd_edpt_xfer(
        rhport, kAudioOutEp, s_audio_out_buffer, sizeof(s_audio_out_buffer));
    if (!rearmed || result != XFER_RESULT_SUCCESS) {
        ESP_LOGW(TAG, "audio OUT rearm=%s result=%d bytes=%lu",
                 rearmed ? "true" : "false",
                 static_cast<int>(result),
                 static_cast<unsigned long>(xferred_bytes));
    }
    return rearmed;
}

const usbd_class_driver_t s_uac1_driver[] = {{
    .name = "ns2_uac1",
    .init = uac1_init,
    .deinit = uac1_deinit,
    .reset = uac1_reset,
    .open = uac1_open,
    .control_xfer_cb = uac1_control_xfer,
    .xfer_cb = uac1_xfer,
    .xfer_isr = nullptr,
    .sof = nullptr,
}};

} // namespace

namespace ns2 {

void usb_uac1_link_driver() {
}

} // namespace ns2

extern "C" usbd_class_driver_t const *usbd_app_driver_get_cb(uint8_t *driver_count) {
    *driver_count = 1;
    return s_uac1_driver;
}
