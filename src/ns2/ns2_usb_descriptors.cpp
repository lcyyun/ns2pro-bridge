#include "ns2_usb.h"

// USB identity, report IDs, and vendor/status interface shape are based on
// NS2Pro/Switch 2 Pro USB experiments from y700-switch2-pro-bridge.

#include <cstring>

#include "tusb.h"

namespace {

constexpr uint16_t USB_VID_NINTENDO = 0x057e;
constexpr uint16_t USB_PID_NINTENDO = 0x2069;
constexpr uint16_t USB_BCD = 0x0200;
constexpr uint8_t USB_SWITCH2_VENDOR_INTERFACE = 1;
constexpr uint8_t USB_SWITCH2_MS_VENDOR_CODE = 0xcd;
constexpr uint16_t SWITCH2_MS_OS_10_COMPAT_ID_LEN = 0x28;
constexpr uint16_t SWITCH2_MS_OS_10_PROPERTY_LEN = 0x8e;
constexpr uint16_t SWITCH2_MS_OS_20_DESC_LEN = 0xB2;
constexpr uint8_t SWITCH2_MS_OS_10_STRING_INDEX = 0xee;
constexpr uint16_t SWITCH2_BOS_TOTAL_LEN = TUD_BOS_DESC_LEN + TUD_BOS_MICROSOFT_OS_DESC_LEN;

constexpr uint8_t EPNUM_HID_OUT = 0x01;
constexpr uint8_t EPNUM_HID_IN = 0x81;
constexpr uint8_t EPNUM_VENDOR_OUT = 0x02;
constexpr uint8_t EPNUM_VENDOR_IN = 0x82;
constexpr uint8_t ITF_NUM_HID = 0;
constexpr uint8_t ITF_NUM_VENDOR = USB_SWITCH2_VENDOR_INTERFACE;
constexpr uint8_t ITF_NUM_TOTAL = 2;
constexpr uint16_t CONFIG_TOTAL_LEN = TUD_CONFIG_DESC_LEN + TUD_HID_INOUT_DESC_LEN + TUD_VENDOR_DESC_LEN;
constexpr uint8_t HID_POLL_INTERVAL_MS = 1;
constexpr uint16_t VENDOR_BULK_PACKET_SIZE = 64;
constexpr uint8_t CONFIG_ATTR_NINTENDO = 0;
constexpr uint16_t CONFIG_POWER_MA_NINTENDO = 500;

#define TUD_VENDOR_INOUT_DESCRIPTOR(_itfnum, _stridx, _epin, _epout, _epsize) \
    9, TUSB_DESC_INTERFACE, _itfnum, 0, 2, TUSB_CLASS_VENDOR_SPECIFIC, 0x00, 0x00, _stridx, \
    7, TUSB_DESC_ENDPOINT, _epin, TUSB_XFER_BULK, U16_TO_U8S_LE(_epsize), 0, \
    7, TUSB_DESC_ENDPOINT, _epout, TUSB_XFER_BULK, U16_TO_U8S_LE(_epsize), 0

#define TUD_HID_NS2_INOUT_DESCRIPTOR(_itfnum, _stridx, _boot_protocol, _report_desc_len, _epin, _epout, _epsize, _ep_interval) \
    9, TUSB_DESC_INTERFACE, _itfnum, 0, 2, TUSB_CLASS_HID, (uint8_t)((_boot_protocol) ? (uint8_t)HID_SUBCLASS_BOOT : 0), _boot_protocol, _stridx, \
    9, HID_DESC_TYPE_HID, U16_TO_U8S_LE(0x0101), 0, 1, HID_DESC_TYPE_REPORT, U16_TO_U8S_LE(_report_desc_len), \
    7, TUSB_DESC_ENDPOINT, _epin, TUSB_XFER_INTERRUPT, U16_TO_U8S_LE(_epsize), _ep_interval, \
    7, TUSB_DESC_ENDPOINT, _epout, TUSB_XFER_INTERRUPT, U16_TO_U8S_LE(_epsize), _ep_interval

enum {
    STRID_LANGID = 0,
    STRID_MANUFACTURER,
    STRID_PRODUCT,
    STRID_SERIAL,
    STRID_CONFIG,
    STRID_HID_INTERFACE,
    STRID_EMPTY,
    STRID_VENDOR_INTERFACE,
};

const uint8_t desc_hid_report_nintendo[] = {
    0x06, 0x00, 0xff, 0x09, 0x01, 0xa1, 0x01, 0x15, 0x00, 0x26, 0xff, 0x00,
    0x75, 0x08, 0x85, NS2_USB_NINTENDO_INPUT_REPORT_ID, 0x95, 0x3f, 0x09, 0x01,
    0x81, 0x02, 0x85, NS2_USB_NINTENDO_OUTPUT_REPORT_ID, 0x95, 0x3f, 0x09, 0x01,
    0x91, 0x02, 0x85, NS2_USB_MANAGER_FEATURE_REPORT_ID, 0x95, 0x3f, 0x09, 0x01,
    0xb1, 0x02, 0xc0,
};

const tusb_desc_device_t desc_device_nintendo = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = USB_BCD,
    .bDeviceClass = 0x00,
    .bDeviceSubClass = 0x00,
    .bDeviceProtocol = 0x00,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = USB_VID_NINTENDO,
    .idProduct = USB_PID_NINTENDO,
    .bcdDevice = 0x0104,
    .iManufacturer = STRID_MANUFACTURER,
    .iProduct = STRID_PRODUCT,
    .iSerialNumber = STRID_SERIAL,
    .bNumConfigurations = 0x01,
};

const uint8_t desc_configuration_nintendo[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, STRID_CONFIG, CONFIG_TOTAL_LEN, CONFIG_ATTR_NINTENDO, CONFIG_POWER_MA_NINTENDO),
    TUD_HID_NS2_INOUT_DESCRIPTOR(ITF_NUM_HID,
                                 STRID_HID_INTERFACE,
                                 HID_ITF_PROTOCOL_NONE,
                                 sizeof(desc_hid_report_nintendo),
                                 EPNUM_HID_IN,
                                 EPNUM_HID_OUT,
                                 NS2_USB_NINTENDO_REPORT_SIZE,
                                 HID_POLL_INTERVAL_MS),
    TUD_VENDOR_INOUT_DESCRIPTOR(ITF_NUM_VENDOR,
                                STRID_VENDOR_INTERFACE,
                                EPNUM_VENDOR_IN,
                                EPNUM_VENDOR_OUT,
                                VENDOR_BULK_PACKET_SIZE),
};

const char *string_desc[] = {
    "",
    "Nintendo Co., Ltd.",
    "Nintendo Switch Pro Controller",
    "HA2F83JF",
    "Nintendo Switch Pro Controller",
    "HID Interface",
    "",
    "Nintendo Switch 2 bulk",
};

alignas(2) const uint8_t ms_os_10_string_descriptor[] = {
    0x12, TUSB_DESC_STRING,
    'M', 0x00, 'S', 0x00, 'F', 0x00, 'T', 0x00,
    '1', 0x00, '0', 0x00, '0', 0x00,
    USB_SWITCH2_MS_VENDOR_CODE, 0x00,
};

const uint8_t ms_os_10_compat_id_descriptor[] = {
    U32_TO_U8S_LE(SWITCH2_MS_OS_10_COMPAT_ID_LEN),
    U16_TO_U8S_LE(0x0100),
    U16_TO_U8S_LE(0x0004),
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    USB_SWITCH2_VENDOR_INTERFACE, 0x01,
    'W', 'I', 'N', 'U', 'S', 'B', 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

const uint8_t ms_os_10_property_descriptor[] = {
    U32_TO_U8S_LE(SWITCH2_MS_OS_10_PROPERTY_LEN),
    U16_TO_U8S_LE(0x0100),
    U16_TO_U8S_LE(0x0005),
    U16_TO_U8S_LE(0x0001),

    U32_TO_U8S_LE(SWITCH2_MS_OS_10_PROPERTY_LEN - 0x0a),
    U32_TO_U8S_LE(0x00000001),
    U16_TO_U8S_LE(0x0028),
    'D', 0x00, 'e', 0x00, 'v', 0x00, 'i', 0x00, 'c', 0x00, 'e', 0x00,
    'I', 0x00, 'n', 0x00, 't', 0x00, 'e', 0x00, 'r', 0x00, 'f', 0x00,
    'a', 0x00, 'c', 0x00, 'e', 0x00, 'G', 0x00, 'U', 0x00, 'I', 0x00,
    'D', 0x00, 0x00, 0x00,
    U32_TO_U8S_LE(0x004e),
    '{', 0x00, '6', 0x00, 'F', 0x00, '1', 0x00, '3', 0x00, '7', 0x00,
    '2', 0x00, '5', 0x00, 'E', 0x00, '-', 0x00, 'E', 0x00, 'F', 0x00,
    '0', 0x00, 'E', 0x00, '-', 0x00, '4', 0x00, 'F', 0x00, 'D', 0x00,
    '3', 0x00, '-', 0x00, 'A', 0x00, 'E', 0x00, '5', 0x00, 'F', 0x00,
    '-', 0x00, 'B', 0x00, '2', 0x00, 'D', 0x00, 'E', 0x00, '9', 0x00,
    '8', 0x00, '9', 0x00, 'E', 0x00, 'C', 0x00, '8', 0x00, '2', 0x00,
    '5', 0x00, '}', 0x00, 0x00, 0x00,
};

const uint8_t bos_descriptor[] = {
    TUD_BOS_DESCRIPTOR(SWITCH2_BOS_TOTAL_LEN, 1),
    TUD_BOS_MS_OS_20_DESCRIPTOR(SWITCH2_MS_OS_20_DESC_LEN, USB_SWITCH2_MS_VENDOR_CODE),
};

const uint8_t ms_os_20_descriptor[] = {
    U16_TO_U8S_LE(0x000A), U16_TO_U8S_LE(MS_OS_20_SET_HEADER_DESCRIPTOR),
    U32_TO_U8S_LE(0x06030000), U16_TO_U8S_LE(SWITCH2_MS_OS_20_DESC_LEN),

    U16_TO_U8S_LE(0x0008), U16_TO_U8S_LE(MS_OS_20_SUBSET_HEADER_CONFIGURATION),
    0, 0, U16_TO_U8S_LE(SWITCH2_MS_OS_20_DESC_LEN - 0x0A),

    U16_TO_U8S_LE(0x0008), U16_TO_U8S_LE(MS_OS_20_SUBSET_HEADER_FUNCTION),
    USB_SWITCH2_VENDOR_INTERFACE, 0,
    U16_TO_U8S_LE(SWITCH2_MS_OS_20_DESC_LEN - 0x0A - 0x08),

    U16_TO_U8S_LE(0x0014), U16_TO_U8S_LE(MS_OS_20_FEATURE_COMPATBLE_ID),
    'W', 'I', 'N', 'U', 'S', 'B', 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    U16_TO_U8S_LE(SWITCH2_MS_OS_20_DESC_LEN - 0x0A - 0x08 - 0x08 - 0x14),
    U16_TO_U8S_LE(MS_OS_20_FEATURE_REG_PROPERTY),
    U16_TO_U8S_LE(0x0007), U16_TO_U8S_LE(0x002A),
    'D', 0x00, 'e', 0x00, 'v', 0x00, 'i', 0x00, 'c', 0x00, 'e', 0x00,
    'I', 0x00, 'n', 0x00, 't', 0x00, 'e', 0x00, 'r', 0x00, 'f', 0x00,
    'a', 0x00, 'c', 0x00, 'e', 0x00, 'G', 0x00, 'U', 0x00, 'I', 0x00,
    'D', 0x00, 's', 0x00, 0x00, 0x00,
    U16_TO_U8S_LE(0x0050),
    '{', 0x00, '6', 0x00, 'F', 0x00, '1', 0x00, '3', 0x00, '7', 0x00,
    '2', 0x00, '5', 0x00, 'E', 0x00, '-', 0x00, 'E', 0x00, 'F', 0x00,
    '0', 0x00, 'E', 0x00, '-', 0x00, '4', 0x00, 'F', 0x00, 'D', 0x00,
    '3', 0x00, '-', 0x00, 'A', 0x00, 'E', 0x00, '5', 0x00, 'F', 0x00,
    '-', 0x00, 'B', 0x00, '2', 0x00, 'D', 0x00, 'E', 0x00, '9', 0x00,
    '8', 0x00, '9', 0x00, 'E', 0x00, 'C', 0x00, '8', 0x00, '2', 0x00,
    '5', 0x00, '}', 0x00, 0x00, 0x00, 0x00, 0x00,
};

uint16_t desc_str[32 + 1];

} // namespace

uint8_t const *tud_descriptor_device_cb(void) {
    return reinterpret_cast<uint8_t const *>(&desc_device_nintendo);
}

uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance) {
    (void)instance;
    return desc_hid_report_nintendo;
}

uint8_t const *tud_descriptor_configuration_cb(uint8_t index) {
    (void)index;
    return desc_configuration_nintendo;
}

uint8_t const *tud_descriptor_bos_cb(void) {
    return bos_descriptor;
}

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void)langid;

    if (index == SWITCH2_MS_OS_10_STRING_INDEX) {
        return reinterpret_cast<uint16_t const *>(ms_os_10_string_descriptor);
    }

    size_t chr_count = 0;
    if (index == STRID_LANGID) {
        desc_str[1] = 0x0409;
        chr_count = 1;
    } else {
        if (index >= sizeof(string_desc) / sizeof(string_desc[0])) {
            return nullptr;
        }
        const char *str = string_desc[index];
        chr_count = strlen(str);
        const size_t max_count = sizeof(desc_str) / sizeof(desc_str[0]) - 1;
        if (chr_count > max_count) {
            chr_count = max_count;
        }
        for (size_t i = 0; i < chr_count; i++) {
            desc_str[1 + i] = static_cast<uint8_t>(str[i]);
        }
    }

    desc_str[0] = static_cast<uint16_t>((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));
    return desc_str;
}

bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request) {
    if (stage != CONTROL_STAGE_SETUP || !request) {
        return true;
    }

    if (request->bmRequestType_bit.type == TUSB_REQ_TYPE_VENDOR &&
        request->bRequest == USB_SWITCH2_MS_VENDOR_CODE) {
        if (request->wIndex == 0x0004) {
            return tud_control_xfer(rhport,
                                    request,
                                    const_cast<uint8_t *>(ms_os_10_compat_id_descriptor),
                                    sizeof(ms_os_10_compat_id_descriptor));
        }
        if (request->wIndex == 0x0005) {
            return tud_control_xfer(rhport,
                                    request,
                                    const_cast<uint8_t *>(ms_os_10_property_descriptor),
                                    sizeof(ms_os_10_property_descriptor));
        }
        if (request->wIndex == 0x0007) {
            return tud_control_xfer(rhport,
                                    request,
                                    const_cast<uint8_t *>(ms_os_20_descriptor),
                                    sizeof(ms_os_20_descriptor));
        }
    }

    return false;
}
