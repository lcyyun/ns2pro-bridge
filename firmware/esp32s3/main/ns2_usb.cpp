#include "ns2_usb.h"

#include "ns2_ble.h"
#include "ns2_config.h"
#include "ns2_input.h"
#include "ns2_protocol.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "class/audio/audio.h"
#include "class/hid/hid_device.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "tinyusb.h"
#include "tinyusb_default_config.h"
#include "tusb.h"

namespace ns2 {
namespace {

const char *TAG = "ns2-usb";

constexpr uint8_t kInputReportPayloadLen = 63;
constexpr uint8_t kFeaturePayloadOffset = 11;
constexpr uint16_t kFeatureReplyMax = 1024;
constexpr uint16_t kDefaultReportRateHz = 250;
constexpr uint16_t kMinReportRateHz = 1;
constexpr uint16_t kMaxReportRateHz = 1000;
constexpr uint16_t kDefaultRumbleHoldMs = 140;
constexpr uint16_t kDefaultRumbleTickMs = 12;
constexpr uint8_t kDefaultRumbleStopPackets = 3;
constexpr uint16_t kDefaultRumbleScalePercent = 60;
constexpr uint16_t kHostRumbleHoldMs = 2000;
constexpr uint16_t kDualMotorMaxAmplitude = 640;
constexpr uint16_t kDs5HighFrequencyMin = 0x172;
constexpr uint16_t kDs5HighFrequencyMax = 0x1a8;
constexpr uint16_t kDs5LowFrequencyMin = 0x108;
constexpr uint16_t kDs5LowFrequencyMax = 0x13e;
constexpr float kDs5OutputStrengthScale = 1.44f;
constexpr float kDs5AudioHapticsBaseStrengthScale = 0.5f;
constexpr uint16_t kDs5HapticsSampleRateHz = 3000;
constexpr uint8_t kDs5HapticsBufferLen = 64;
constexpr uint8_t kDs5HapticsWindowHop = 36;
constexpr uint8_t kDs5HapticsLowBinMin = 2;
constexpr uint8_t kDs5HapticsLowBinMax = 5;
constexpr uint8_t kDs5HapticsHighBinMin = 6;
constexpr uint8_t kDs5HapticsHighBinMax = 13;
constexpr uint16_t kDs5HapticsMinRms = 96;
constexpr float kDs5SpectralAmplitudeGain = 4.0f;
constexpr uint8_t kDs5AudioDecimation = 16;
constexpr uint16_t kDs5AudioPacketBytes = 384;
constexpr uint8_t kDs5AudioQueueDepth = 16;
constexpr uint8_t kDs5OutputReportId = 0x02;
constexpr uint8_t kDs5RumbleEnableMask = 0x03;
constexpr uint16_t kDs5ImprovedRumbleOffset = 38;
constexpr uint8_t kDs5ImprovedRumbleMask = 0x04;
constexpr size_t kMotionReportOffset = 48;
constexpr uint16_t kStickCenter12 = 2048;
constexpr uint16_t kStickInputFullRange12 = 1200;
constexpr uint32_t kUsbKeepaliveIntervalUs = 50000;
constexpr uint8_t kHidInterface = 0;
constexpr uint8_t kVendorInterface = 1;
constexpr uint8_t kInterfaceCount = 2;
constexpr uint8_t kHidEpOut = 0x01;
constexpr uint8_t kHidEpIn = 0x81;
constexpr uint8_t kVendorEpOut = 0x02;
constexpr uint8_t kVendorEpIn = 0x82;
constexpr uint8_t kXInputEpIn = 0x81;
constexpr uint8_t kXInputEpOut = 0x02;
constexpr uint8_t kManagerHidEpOut = 0x03;
constexpr uint8_t kManagerHidEpIn = 0x83;
constexpr uint8_t kDualSenseAudioOutEp = 0x01;
constexpr uint8_t kDualSenseAudioInEp = 0x82;
constexpr uint8_t kDualSenseHidEpOut = 0x03;
constexpr uint8_t kDualSenseHidEpIn = 0x84;
constexpr uint8_t kDualSenseAudioControlInterface = 0;
constexpr uint8_t kDualSenseAudioOutInterface = 1;
constexpr uint8_t kDualSenseAudioInInterface = 2;
constexpr uint8_t kDualSenseHidInterface = 3;
constexpr uint8_t kDualSenseInterfaceCount = 4;
constexpr uint8_t kVendorInstance = 0;
constexpr uint8_t kGamepadHidInstance = 0;
constexpr uint8_t kManagerHidInstance = 1;
constexpr uint8_t kDualSenseInputReportId = 0x01;
constexpr uint8_t kDualSenseInputPayloadLen = 63;
constexpr uint8_t kUac1SpeakerFeatureUnit = 0x02;
constexpr uint8_t kUac1MicFeatureUnit = 0x05;
constexpr uint8_t kAudio10CsReqSetCur = 0x01;
constexpr uint8_t kAudio10CsReqGetCur = 0x81;
constexpr uint8_t kAudio10CsReqGetMin = 0x82;
constexpr uint8_t kAudio10CsReqGetMax = 0x83;
constexpr uint8_t kAudio10CsReqGetRes = 0x84;
constexpr uint8_t kAudio10FuCtrlMute = 0x01;
constexpr uint8_t kAudio10FuCtrlVolume = 0x02;
constexpr uint8_t kXInputReportLen = 20;
constexpr uint8_t kMsOs10StringIndex = 0xee;
constexpr uint8_t kMsVendorCode = 0xcd;
constexpr uint16_t kMsOs20DescriptorLen = 0xB2;
constexpr uint16_t kMsOs10CompatIdLen = 0x28;
constexpr uint16_t kMsOs10PropertyLen = 0x8e;
constexpr uint16_t kBulkReplyMax = 128;
constexpr uint16_t kBulkPendingMax = 3072;
constexpr uint16_t kFlashReplyFullSpeedLen = 0x50;
constexpr int64_t kHidGuardTimeoutUs = 10LL * 60LL * 1000LL * 1000LL;
constexpr uint64_t kModeComboHoldUs = 2ULL * 1000ULL * 1000ULL;
constexpr uint64_t kFeatureQuietUs = 8ULL * 1000ULL;
constexpr uint32_t kModeComboMask = (1u << 6) | (1u << 16) | (1u << 17); // Plus + Home + Capture

constexpr char kFeatureSetMagic[] = "Y7HID1";
constexpr char kFeatureReplyMagic[] = "Y7HRS1";

const tusb_desc_device_t kDeviceDescriptor = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = kNintendoVid,
    .idProduct = kNintendoPid,
    .bcdDevice = 0x0104,
    .iManufacturer = 0x01,
    .iProduct = 0x02,
    .iSerialNumber = 0x03,
    .bNumConfigurations = 0x01,
};

const tusb_desc_device_t kXInputDeviceDescriptor = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = 0x045e,
    .idProduct = 0x028e,
    .bcdDevice = 0x0572,
    .iManufacturer = 0x01,
    .iProduct = 0x02,
    .iSerialNumber = 0x03,
    .bNumConfigurations = 0x01,
};

const tusb_desc_device_t kDualSenseDeviceDescriptor = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = 0x054c,
    .idProduct = 0x0ce6,
    .bcdDevice = 0x0100,
    .iManufacturer = 0x01,
    .iProduct = 0x02,
    .iSerialNumber = 0x00,
    .bNumConfigurations = 0x01,
};

const uint8_t kHidReportDescriptor[] = {
    0x06, 0x00, 0xff, 0x09, 0x01, 0xa1, 0x01, 0x15, 0x00, 0x26, 0xff, 0x00,
    0x75, 0x08, 0x85, kUsbReportIdInput, 0x95, kInputReportPayloadLen, 0x09, 0x01,
    0x81, 0x02, 0x85, kUsbReportIdOutput, 0x95, kInputReportPayloadLen, 0x09, 0x01,
    0x91, 0x02, 0x85, kUsbReportIdFeature, 0x95, kInputReportPayloadLen, 0x09, 0x01,
    0xb1, 0x02, 0xc0,
};

const uint8_t kDualSenseReportDescriptor[] = {
    0x05, 0x01, 0x09, 0x05, 0xa1, 0x01, 0x85, 0x01, 0x09, 0x30, 0x09, 0x31,
    0x09, 0x32, 0x09, 0x35, 0x09, 0x33, 0x09, 0x34, 0x15, 0x00, 0x26, 0xff,
    0x00, 0x75, 0x08, 0x95, 0x06, 0x81, 0x02, 0x06, 0x00, 0xff, 0x09, 0x20,
    0x95, 0x01, 0x81, 0x02, 0x05, 0x01, 0x09, 0x39, 0x15, 0x00, 0x25, 0x07,
    0x35, 0x00, 0x46, 0x3b, 0x01, 0x65, 0x14, 0x75, 0x04, 0x95, 0x01, 0x81,
    0x42, 0x65, 0x00, 0x05, 0x09, 0x19, 0x01, 0x29, 0x0f, 0x15, 0x00, 0x25,
    0x01, 0x75, 0x01, 0x95, 0x0f, 0x81, 0x02, 0x06, 0x00, 0xff, 0x09, 0x21,
    0x95, 0x0d, 0x81, 0x02, 0x06, 0x00, 0xff, 0x09, 0x22, 0x15, 0x00, 0x26,
    0xff, 0x00, 0x75, 0x08, 0x95, 0x34, 0x81, 0x02, 0x85, 0x02, 0x09, 0x23,
    0x95, 0x2f, 0x91, 0x02, 0x85, 0x05, 0x09, 0x33, 0x95, 0x28, 0xb1, 0x02,
    0x85, 0x08, 0x09, 0x34, 0x95, 0x2f, 0xb1, 0x02, 0x85, 0x09, 0x09, 0x24,
    0x95, 0x13, 0xb1, 0x02, 0x85, 0x0a, 0x09, 0x25, 0x95, 0x1a, 0xb1, 0x02,
    0x85, 0x0b, 0x09, 0x41, 0x95, 0x29, 0xb1, 0x02, 0x85, 0x0c, 0x09, 0x42,
    0x95, 0x29, 0xb1, 0x02, 0x85, 0x20, 0x09, 0x26, 0x95, 0x3f, 0xb1, 0x02,
    0x85, 0x21, 0x09, 0x27, 0x95, 0x04, 0xb1, 0x02, 0x85, 0x22, 0x09, 0x40,
    0x95, 0x3f, 0xb1, 0x02, 0x85, 0x80, 0x09, 0x28, 0x95, 0x3f, 0xb1, 0x02,
    0x85, 0x81, 0x09, 0x29, 0x95, 0x3f, 0xb1, 0x02, 0x85, 0x82, 0x09, 0x2a,
    0x95, 0x09, 0xb1, 0x02, 0x85, 0x83, 0x09, 0x2b, 0x95, 0x3f, 0xb1, 0x02,
    0x85, 0x84, 0x09, 0x2c, 0x95, 0x3f, 0xb1, 0x02, 0x85, 0x85, 0x09, 0x2d,
    0x95, 0x02, 0xb1, 0x02, 0x85, 0xa0, 0x09, 0x2e, 0x95, 0x01, 0xb1, 0x02,
    0x85, 0xe0, 0x09, 0x2f, 0x95, 0x3f, 0xb1, 0x02, 0x85, 0xf0, 0x09, 0x30,
    0x95, 0x3f, 0xb1, 0x02, 0x85, 0xf1, 0x09, 0x31, 0x95, 0x3f, 0xb1, 0x02,
    0x85, 0xf2, 0x09, 0x32, 0x95, 0x0f, 0xb1, 0x02, 0x85, 0xf4, 0x09, 0x35,
    0x95, 0x3f, 0xb1, 0x02, 0x85, 0xf5, 0x09, 0x36, 0x95, 0x03, 0xb1, 0x02,
    0x85, 0xf6, 0x09, 0x37, 0x95, 0x3f, 0xb1, 0x02, 0x85, 0xf7, 0x09, 0x38,
    0x95, 0x3f, 0xb1, 0x02, 0x85, 0xf8, 0x09, 0x39, 0x95, 0x3f, 0xb1, 0x02,
    0x85, 0xf9, 0x09, 0x3a, 0x95, 0x3f, 0xb1, 0x02, 0xc0,
};
static_assert(sizeof(kDualSenseReportDescriptor) == 0x0141);

const char kLangString[] = {0x09, 0x04};

const char *kStringDescriptor[] = {
    kLangString,
    "Nintendo Co., Ltd.",
    "Nintendo Switch Pro Controller",
    "HA2F83JF",
    "Nintendo Switch Pro Controller",
    "HID Interface",
    "",
    "Nintendo Switch 2 bulk",
};

const char *kXInputStringDescriptor[] = {
    kLangString,
    "Microsoft",
    "Controller",
    "1.0",
};

const char *kDualSenseStringDescriptor[] = {
    kLangString,
    "Sony Interactive Entertainment",
    "DualSense Wireless Controller",
    "ESP32S3DS5",
    "DualSense Wireless Controller",
    "HID Interface",
};

#define TUD_VENDOR_INOUT_DESCRIPTOR(_itfnum, _stridx, _epin, _epout, _epsize) \
    9, TUSB_DESC_INTERFACE, _itfnum, 0, 2, TUSB_CLASS_VENDOR_SPECIFIC, 0x00, 0x00, _stridx, \
    7, TUSB_DESC_ENDPOINT, _epin, TUSB_XFER_BULK, U16_TO_U8S_LE(_epsize), 0, \
    7, TUSB_DESC_ENDPOINT, _epout, TUSB_XFER_BULK, U16_TO_U8S_LE(_epsize), 0

#define TUD_HID_NS2_INOUT_DESCRIPTOR(_itfnum, _stridx, _boot_protocol, _report_desc_len, _epin, _epout, _epsize, _ep_interval) \
    9, TUSB_DESC_INTERFACE, _itfnum, 0, 2, TUSB_CLASS_HID, (uint8_t)((_boot_protocol) ? (uint8_t)HID_SUBCLASS_BOOT : 0), _boot_protocol, _stridx, \
    9, HID_DESC_TYPE_HID, U16_TO_U8S_LE(0x0101), 0, 1, HID_DESC_TYPE_REPORT, U16_TO_U8S_LE(_report_desc_len), \
    7, TUSB_DESC_ENDPOINT, _epin, TUSB_XFER_INTERRUPT, U16_TO_U8S_LE(_epsize), _ep_interval, \
    7, TUSB_DESC_ENDPOINT, _epout, TUSB_XFER_INTERRUPT, U16_TO_U8S_LE(_epsize), _ep_interval

constexpr uint16_t kConfigTotalLen = TUD_CONFIG_DESC_LEN + TUD_HID_INOUT_DESC_LEN + TUD_VENDOR_DESC_LEN;
const uint8_t kConfigurationDescriptor[] = {
    TUD_CONFIG_DESCRIPTOR(1, kInterfaceCount, 4, kConfigTotalLen, 0, 500),
    TUD_HID_NS2_INOUT_DESCRIPTOR(kHidInterface, 5, HID_ITF_PROTOCOL_NONE, sizeof(kHidReportDescriptor), kHidEpIn, kHidEpOut, 64, 1),
    TUD_VENDOR_INOUT_DESCRIPTOR(kVendorInterface, 7, kVendorEpIn, kVendorEpOut, 64),
};

constexpr uint16_t kXInputConfigTotalLen = 0x30 + TUD_HID_INOUT_DESC_LEN;
const uint8_t kXInputConfigurationDescriptor[] = {
    TUD_CONFIG_DESCRIPTOR(1, 2, 0, kXInputConfigTotalLen, 0, 500),
    9, TUSB_DESC_INTERFACE, 0, 0, 2, TUSB_CLASS_VENDOR_SPECIFIC, 0x5d, 0x01, 0,
    0x10, 0x21, 0x10, 0x01, 0x01, 0x24, 0x81, 0x14,
    0x03, 0x00, 0x03, 0x13, 0x02, 0x00, 0x03, 0x00,
    7, TUSB_DESC_ENDPOINT, kXInputEpIn, TUSB_XFER_INTERRUPT, U16_TO_U8S_LE(32), 4,
    7, TUSB_DESC_ENDPOINT, kXInputEpOut, TUSB_XFER_INTERRUPT, U16_TO_U8S_LE(32), 8,
    TUD_HID_NS2_INOUT_DESCRIPTOR(1, 0, HID_ITF_PROTOCOL_NONE, sizeof(kHidReportDescriptor), kManagerHidEpIn, kManagerHidEpOut, 64, 4),
};

constexpr uint16_t kDualSenseConfigTotalLen = 0x00e3;
const uint8_t kDualSenseConfigurationDescriptor[] = {
    0x09, TUSB_DESC_CONFIGURATION, U16_TO_U8S_LE(kDualSenseConfigTotalLen),
    kDualSenseInterfaceCount, 0x01, 0x00, 0xc0, 0xfa,

    0x09, TUSB_DESC_INTERFACE,
    kDualSenseAudioControlInterface, 0x00, 0x00,
    TUSB_CLASS_AUDIO, AUDIO_SUBCLASS_CONTROL, 0x00, 0x00,
    0x0a, 0x24, 0x01, 0x00, 0x01, 0x49, 0x00, 0x02,
    kDualSenseAudioOutInterface, kDualSenseAudioInInterface,

    0x0c, 0x24, 0x02, 0x01, 0x01, 0x01, 0x06, 0x04, 0x33, 0x00, 0x00, 0x00,
    0x0c, 0x24, 0x06, kUac1SpeakerFeatureUnit, 0x01, 0x01,
    0x03, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x09, 0x24, 0x03, 0x03, 0x01, 0x03, 0x04, kUac1SpeakerFeatureUnit, 0x00,
    0x0c, 0x24, 0x02, 0x04, 0x02, 0x04, 0x03, 0x02, 0x03, 0x00, 0x00, 0x00,
    0x09, 0x24, 0x06, kUac1MicFeatureUnit, 0x04, 0x01,
    0x03, 0x00, 0x00,
    0x09, 0x24, 0x03, 0x06, 0x01, 0x01, 0x01, kUac1MicFeatureUnit, 0x00,

    0x09, TUSB_DESC_INTERFACE,
    kDualSenseAudioOutInterface, 0x00, 0x00,
    TUSB_CLASS_AUDIO, AUDIO_SUBCLASS_STREAMING, 0x00, 0x00,
    0x09, TUSB_DESC_INTERFACE,
    kDualSenseAudioOutInterface, 0x01, 0x01,
    TUSB_CLASS_AUDIO, AUDIO_SUBCLASS_STREAMING, 0x00, 0x00,
    0x07, 0x24, 0x01, 0x01, 0x01, 0x01, 0x00,
    0x0b, 0x24, 0x02, 0x01, 0x04, 0x02, 0x10, 0x01, 0x80, 0xbb, 0x00,
    0x09, TUSB_DESC_ENDPOINT, kDualSenseAudioOutEp, 0x09, 0x80, 0x01, 0x01, 0x00, 0x00,
    0x07, 0x25, 0x01, 0x00, 0x00, 0x00, 0x00,

    0x09, TUSB_DESC_INTERFACE,
    kDualSenseAudioInInterface, 0x00, 0x00,
    TUSB_CLASS_AUDIO, AUDIO_SUBCLASS_STREAMING, 0x00, 0x00,
    0x09, TUSB_DESC_INTERFACE,
    kDualSenseAudioInInterface, 0x01, 0x01,
    TUSB_CLASS_AUDIO, AUDIO_SUBCLASS_STREAMING, 0x00, 0x00,
    0x07, 0x24, 0x01, 0x06, 0x01, 0x01, 0x00,
    0x0b, 0x24, 0x02, 0x01, 0x02, 0x02, 0x10, 0x01, 0x80, 0xbb, 0x00,
    0x09, TUSB_DESC_ENDPOINT, kDualSenseAudioInEp, 0x05, 0xc0, 0x00, 0x01, 0x00, 0x00,
    0x07, 0x25, 0x01, 0x00, 0x00, 0x00, 0x00,

    0x09, TUSB_DESC_INTERFACE,
    kDualSenseHidInterface, 0x00, 0x02,
    TUSB_CLASS_HID, 0x00, 0x00, 0x00,
    0x09, HID_DESC_TYPE_HID, 0x11, 0x01, 0x00, 0x01,
    HID_DESC_TYPE_REPORT, U16_TO_U8S_LE(sizeof(kDualSenseReportDescriptor)),
    0x07, TUSB_DESC_ENDPOINT, kDualSenseHidEpIn, TUSB_XFER_INTERRUPT, 0x40, 0x00, 0x01,
    0x07, TUSB_DESC_ENDPOINT, kDualSenseHidEpOut, TUSB_XFER_INTERRUPT, 0x40, 0x00, 0x01,
};
static_assert(sizeof(kDualSenseConfigurationDescriptor) == kDualSenseConfigTotalLen);

const uint8_t kMsOs10StringDescriptor[] = {
    0x12, TUSB_DESC_STRING,
    'M', 0x00, 'S', 0x00, 'F', 0x00, 'T', 0x00,
    '1', 0x00, '0', 0x00, '0', 0x00,
    kMsVendorCode, 0x00,
};

const uint8_t kMsOs10CompatIdDescriptor[] = {
    U32_TO_U8S_LE(kMsOs10CompatIdLen),
    U16_TO_U8S_LE(0x0100),
    U16_TO_U8S_LE(0x0004),
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    kVendorInterface, 0x01,
    'W', 'I', 'N', 'U', 'S', 'B', 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

const uint8_t kMsOs10PropertyDescriptor[] = {
    U32_TO_U8S_LE(kMsOs10PropertyLen),
    U16_TO_U8S_LE(0x0100),
    U16_TO_U8S_LE(0x0005),
    U16_TO_U8S_LE(0x0001),

    U32_TO_U8S_LE(kMsOs10PropertyLen - 0x0a),
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

const uint8_t kBosDescriptor[] = {
    TUD_BOS_DESCRIPTOR(TUD_BOS_DESC_LEN + TUD_BOS_MICROSOFT_OS_DESC_LEN, 1),
    TUD_BOS_MS_OS_20_DESCRIPTOR(kMsOs20DescriptorLen, kMsVendorCode),
};

const uint8_t kMsOs20Descriptor[] = {
    U16_TO_U8S_LE(0x000A), U16_TO_U8S_LE(MS_OS_20_SET_HEADER_DESCRIPTOR),
    U32_TO_U8S_LE(0x06030000), U16_TO_U8S_LE(kMsOs20DescriptorLen),

    U16_TO_U8S_LE(0x0008), U16_TO_U8S_LE(MS_OS_20_SUBSET_HEADER_CONFIGURATION),
    0, 0, U16_TO_U8S_LE(kMsOs20DescriptorLen - 0x0A),

    U16_TO_U8S_LE(0x0008), U16_TO_U8S_LE(MS_OS_20_SUBSET_HEADER_FUNCTION),
    kVendorInterface, 0,
    U16_TO_U8S_LE(kMsOs20DescriptorLen - 0x0A - 0x08),

    U16_TO_U8S_LE(0x0014), U16_TO_U8S_LE(MS_OS_20_FEATURE_COMPATBLE_ID),
    'W', 'I', 'N', 'U', 'S', 'B', 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    U16_TO_U8S_LE(kMsOs20DescriptorLen - 0x0A - 0x08 - 0x08 - 0x14),
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

struct UsbState {
    bool started = false;
    bool mounted = false;
    bool suspended = false;
    bool raw_passthrough = false;
    bool web_parse_reports = true;
    OutputMode output_mode = OutputMode::Nintendo;
    OutputMode pending_output_mode = OutputMode::Nintendo;
    bool pending_output_mode_valid = false;
    bool rumble_enabled = true;
    bool debug_force_a = false;
    bool debug_live_log = false;
    bool rumble_active = false;
    uint16_t rumble_scale_percent = kDefaultRumbleScalePercent;
    uint16_t rumble_hold_ms = kDefaultRumbleHoldMs;
    uint16_t rumble_tick_ms = kDefaultRumbleTickMs;
    uint8_t rumble_stop_packets = kDefaultRumbleStopPackets;
    uint8_t rumble_left[3][5] = {};
    uint8_t rumble_right[3][5] = {};
    uint64_t rumble_until_us = 0;
    uint64_t rumble_next_tick_us = 0;
    uint8_t rumble_stop_pending = 0;
    uint8_t rumble_packet_id = 0;
    uint32_t rumble_updates = 0;
    uint32_t rumble_writes = 0;
    uint32_t rumble_stops = 0;
    uint32_t rumble_errors = 0;
    uint32_t rumble_switch_reports = 0;
    uint32_t rumble_ds5_reports = 0;
    uint32_t rumble_xinput_reports = 0;
    uint32_t rumble_xinput_out_logged = 0;
    uint32_t rumble_dual_motor_reports = 0;
    uint32_t rumble_stream_logged = 0;
    uint8_t rumble_last_left_heavy = 0;
    uint8_t rumble_last_right_light = 0;
    char rumble_last_source[16] = "none";
    uint8_t ds5_regular_right = 0;
    uint8_t ds5_regular_left = 0;
    uint8_t ds5_haptic_right_low = 0;
    uint8_t ds5_haptic_right_high = 0;
    uint8_t ds5_haptic_left_low = 0;
    uint8_t ds5_haptic_left_high = 0;
    int16_t ds5_haptics_left_samples[kDs5HapticsBufferLen] = {};
    int16_t ds5_haptics_right_samples[kDs5HapticsBufferLen] = {};
    uint8_t ds5_haptics_sample_pos = 0;
    uint16_t ds5_haptic_left_low_freq = 0x112;
    uint16_t ds5_haptic_left_high_freq = 0x187;
    uint16_t ds5_haptic_right_low_freq = 0x112;
    uint16_t ds5_haptic_right_high_freq = 0x187;
    uint16_t ds5_haptic_left_low_rms = 0;
    uint16_t ds5_haptic_left_high_rms = 0;
    uint16_t ds5_haptic_right_low_rms = 0;
    uint16_t ds5_haptic_right_high_rms = 0;
    uint8_t ds5_audio_decim_count = 0;
    int32_t ds5_audio_left_sum = 0;
    int32_t ds5_audio_right_sum = 0;
    uint8_t audio_mute[2] = {};
    int16_t audio_volume_256[2] = {-100 * 256, 0};
    uint32_t audio_set_interface_count = 0;
    uint8_t audio_last_interface = 0;
    uint8_t audio_last_alt = 0;
    uint32_t audio_out_packets = 0;
    uint32_t audio_out_bytes = 0;
    uint16_t audio_last_read = 0;
    uint32_t audio_logged = 0;
    uint16_t report_rate_hz = kDefaultReportRateHz;
    uint32_t report_interval_us = 1000000u / kDefaultReportRateHz;
    uint64_t next_report_us = 0;
    uint32_t last_reported_input_updates = 0;
    uint8_t counter = 0;
    uint32_t reports_sent = 0;
    uint32_t reports_failed = 0;
    uint64_t report_rate_sample_us = 0;
    uint32_t report_rate_sample_count = 0;
    uint32_t report_submit_hz = 0;
    uint32_t report_last_gap_us = 0;
    uint32_t report_max_gap_us = 0;
    uint64_t report_last_sent_us = 0;
    uint32_t reports_completed = 0;
    uint64_t report_complete_sample_us = 0;
    uint32_t report_complete_sample_count = 0;
    uint32_t report_complete_hz = 0;
    uint32_t report_complete_last_gap_us = 0;
    uint32_t report_complete_max_gap_us = 0;
    uint64_t report_last_complete_us = 0;
    uint32_t parsed_reports = 0;
    uint32_t raw_reports = 0;
    uint32_t debug_live_logged = 0;
    uint32_t hid_out_count = 0;
    uint8_t hid_last_report_id = 0;
    uint8_t hid_last_type = 0;
    uint16_t hid_last_len = 0;
    uint32_t vendor_out_count = 0;
    uint32_t vendor_in_count = 0;
    uint32_t vendor_in_done_count = 0;
    uint32_t vendor_last_sent_bytes = 0;
    uint16_t vendor_last_rx_len = 0;
    uint16_t vendor_last_tx_len = 0;
    uint8_t vendor_last_cmd = 0;
    uint8_t vendor_last_arg = 0;
    uint32_t vendor_last_address = 0;
    uint8_t vendor_pending[kBulkPendingMax] = {};
    uint16_t vendor_pending_len = 0;
    uint16_t vendor_pending_offset = 0;
    uint8_t vendor_pending_itf = 0;
    bool hid_guard_active = false;
    bool hid_guard_done = false;
    bool hid_guard_release_after_tx = false;
    int64_t hid_guard_started_us = 0;
    uint32_t feature_set_count = 0;
    uint32_t feature_get_count = 0;
    uint64_t feature_quiet_until_us = 0;
    uint8_t feature_reply[kFeatureReplyMax] = {};
    uint16_t feature_reply_len = 0;
    uint16_t feature_reply_offset = 0;
    char feature_last_command[64] = {};
    uint64_t mode_combo_started_us = 0;
    bool mode_combo_fired = false;
};

UsbState s_usb;

struct Ds5AudioPacket {
    uint16_t len = 0;
    uint8_t data[kDs5AudioPacketBytes] = {};
};

StaticQueue_t s_ds5_audio_queue_storage;
uint8_t s_ds5_audio_queue_buffer[kDs5AudioQueueDepth * sizeof(Ds5AudioPacket)] = {};
QueueHandle_t s_ds5_audio_queue = nullptr;

void usb_event_cb(tinyusb_event_t *event, void *arg);

OutputMode saved_output_mode() {
    return s_usb.pending_output_mode_valid ? s_usb.pending_output_mode : s_usb.output_mode;
}

bool output_mode_restart_required() {
    return saved_output_mode() != s_usb.output_mode;
}

struct StickAxes12 {
    uint16_t x = kStickCenter12;
    uint16_t y = kStickCenter12;
};

const tusb_desc_device_t *current_device_descriptor() {
    switch (s_usb.output_mode) {
    case OutputMode::XInput:
        return &kXInputDeviceDescriptor;
    case OutputMode::DualSense:
        return &kDualSenseDeviceDescriptor;
    case OutputMode::Nintendo:
    default:
        return &kDeviceDescriptor;
    }
}

const char **current_string_descriptor() {
    switch (s_usb.output_mode) {
    case OutputMode::XInput:
        return kXInputStringDescriptor;
    case OutputMode::DualSense:
        return kDualSenseStringDescriptor;
    case OutputMode::Nintendo:
    default:
        return kStringDescriptor;
    }
}

size_t current_string_descriptor_count() {
    switch (s_usb.output_mode) {
    case OutputMode::XInput:
        return sizeof(kXInputStringDescriptor) / sizeof(kXInputStringDescriptor[0]);
    case OutputMode::DualSense:
        return sizeof(kDualSenseStringDescriptor) / sizeof(kDualSenseStringDescriptor[0]);
    case OutputMode::Nintendo:
    default:
        return sizeof(kStringDescriptor) / sizeof(kStringDescriptor[0]);
    }
}

const uint8_t *current_configuration_descriptor() {
    switch (s_usb.output_mode) {
    case OutputMode::XInput:
        return kXInputConfigurationDescriptor;
    case OutputMode::DualSense:
        return kDualSenseConfigurationDescriptor;
    case OutputMode::Nintendo:
    default:
        return kConfigurationDescriptor;
    }
}

const char *skip_spaces(const char *text) {
    while (text != nullptr && std::isspace(static_cast<unsigned char>(*text))) {
        ++text;
    }
    return text != nullptr ? text : "";
}

bool command_is(const char *command, const char *expected) {
    return std::strcmp(skip_spaces(command), expected) == 0;
}

bool command_has_prefix(const char *command, const char *prefix) {
    command = skip_spaces(command);
    const size_t len = std::strlen(prefix);
    return std::strncmp(command, prefix, len) == 0 &&
           (command[len] == 0 || std::isspace(static_cast<unsigned char>(command[len])));
}

bool parse_next_uint(const char **cursor, uint32_t *out) {
    const char *p = skip_spaces(*cursor);
    if (*p == 0) {
        return false;
    }
    char *end = nullptr;
    const unsigned long value = std::strtoul(p, &end, 10);
    if (end == p) {
        return false;
    }
    *out = static_cast<uint32_t>(value);
    *cursor = end;
    return true;
}

bool parse_output_mode(const char *text, OutputMode *out) {
    text = skip_spaces(text);
    if (out == nullptr || text == nullptr) {
        return false;
    }
    if (std::strcmp(text, "nintendo") == 0 ||
        std::strcmp(text, "switch") == 0 ||
        std::strcmp(text, "pro2") == 0) {
        *out = OutputMode::Nintendo;
        return true;
    }
    if (std::strcmp(text, "xinput") == 0 ||
        std::strcmp(text, "xbox") == 0) {
        *out = OutputMode::XInput;
        return true;
    }
    if (std::strcmp(text, "dualsense") == 0 ||
        std::strcmp(text, "ds5") == 0) {
        *out = OutputMode::DualSense;
        return true;
    }
    return false;
}

uint32_t command_address(const uint8_t *cmd, uint16_t cmd_len) {
    if (cmd == nullptr || cmd_len < 16) {
        return 0;
    }
    return static_cast<uint32_t>(cmd[12]) |
           (static_cast<uint32_t>(cmd[13]) << 8) |
           (static_cast<uint32_t>(cmd[14]) << 16) |
           (static_cast<uint32_t>(cmd[15]) << 24);
}

void write_bytes(uint8_t *out, size_t out_len, size_t offset, const uint8_t *data, size_t data_len) {
    if (out == nullptr || data == nullptr || offset >= out_len) {
        return;
    }
    const size_t copy_len = std::min(data_len, out_len - offset);
    std::memcpy(out + offset, data, copy_len);
}

void pack_stick(uint8_t report[kInputReportPayloadLen], size_t offset, uint16_t x, uint16_t y) {
    x = std::min<uint16_t>(x, 4095);
    y = std::min<uint16_t>(y, 4095);
    report[offset] = static_cast<uint8_t>(x & 0xff);
    report[offset + 1] = static_cast<uint8_t>(((x >> 8) & 0x0f) | ((y & 0x0f) << 4));
    report[offset + 2] = static_cast<uint8_t>((y >> 4) & 0xff);
}

void pack_stick_calibration(uint8_t out[9]) {
    pack_stick(out, 0, 2048, 2048);
    pack_stick(out, 3, 2048, 2048);
    pack_stick(out, 6, 2048, 2048);
}

uint16_t normalize_stick_axis(uint16_t value) {
    int32_t delta = static_cast<int32_t>(value) - kStickCenter12;
    if (delta == 0) {
        return kStickCenter12;
    }

    const int32_t rounding = delta > 0 ?
        kStickInputFullRange12 / 2 :
        -(kStickInputFullRange12 / 2);
    const int32_t scaled = (delta * kStickCenter12 + rounding) / kStickInputFullRange12;
    const int32_t mapped = static_cast<int32_t>(kStickCenter12) + scaled;
    return static_cast<uint16_t>(std::max<int32_t>(0, std::min<int32_t>(4095, mapped)));
}

uint16_t clamp_stick12(int32_t value) {
    return static_cast<uint16_t>(std::max<int32_t>(0, std::min<int32_t>(4095, value)));
}

StickAxes12 normalize_stick_pair(uint16_t x, uint16_t y) {
    const uint16_t nx = normalize_stick_axis(x);
    const uint16_t ny = normalize_stick_axis(y);
    int32_t dx = static_cast<int32_t>(nx) - kStickCenter12;
    int32_t dy = static_cast<int32_t>(ny) - kStickCenter12;

    constexpr float kRadius = 2047.0f;
    const float mag_sq = static_cast<float>(dx * dx + dy * dy);
    if (mag_sq > kRadius * kRadius) {
        const float scale = kRadius / std::sqrt(mag_sq);
        dx = static_cast<int32_t>(std::lround(static_cast<float>(dx) * scale));
        dy = static_cast<int32_t>(std::lround(static_cast<float>(dy) * scale));
    }

    return {
        clamp_stick12(static_cast<int32_t>(kStickCenter12) + dx),
        clamp_stick12(static_cast<int32_t>(kStickCenter12) + dy),
    };
}

bool button_pressed(const InputSnapshot &input, uint8_t button) {
    return (input.buttons & (1u << button)) != 0;
}

OutputMode next_output_mode(OutputMode mode) {
    switch (mode) {
    case OutputMode::Nintendo:
        return OutputMode::XInput;
    case OutputMode::XInput:
        return OutputMode::DualSense;
    case OutputMode::DualSense:
    default:
        return OutputMode::Nintendo;
    }
}

void check_mode_combo(const InputSnapshot *input, bool live, uint64_t now) {
    if (!live || input == nullptr || (input->buttons & kModeComboMask) != kModeComboMask) {
        s_usb.mode_combo_started_us = 0;
        s_usb.mode_combo_fired = false;
        return;
    }

    if (s_usb.mode_combo_started_us == 0) {
        s_usb.mode_combo_started_us = now;
        return;
    }
    if (s_usb.mode_combo_fired || now - s_usb.mode_combo_started_us < kModeComboHoldUs) {
        return;
    }

    const OutputMode next = next_output_mode(s_usb.output_mode);
    config_set_output_mode(next);
    config_save();
    s_usb.mode_combo_fired = true;
    ESP_LOGW(TAG, "mode combo held; switching USB mode %s -> %s",
             config_output_mode_name(s_usb.output_mode),
             config_output_mode_name(next));
    esp_restart();
}

void fill_neutral_input(uint8_t report[kInputReportPayloadLen]) {
    std::memset(report, 0, kInputReportPayloadLen);
    report[0] = s_usb.counter++;
    report[1] = 0x20;
    pack_stick(report, 10, 2048, 2048);
    pack_stick(report, 13, 2048, 2048);
}

void fill_input_report(const InputSnapshot *input, uint8_t report[kInputReportPayloadLen]) {
    fill_neutral_input(report);
    if (s_usb.debug_force_a) {
        report[4] |= 0x08;
    }
    if (input == nullptr || !input->valid) {
        return;
    }
    if (s_usb.raw_passthrough &&
        input->kind == InputReportKind::Fd2 &&
        input->raw_valid &&
        input->raw_len == kInputReportPayloadLen) {
        std::memcpy(report, input->raw, kInputReportPayloadLen);
        s_usb.raw_reports++;
        return;
    }

    if (button_pressed(*input, 2)) report[4] |= 0x01;  // Y
    if (button_pressed(*input, 3)) report[4] |= 0x02;  // X
    if (button_pressed(*input, 0)) report[4] |= 0x04;  // B
    if (button_pressed(*input, 1)) report[4] |= 0x08;  // A
    if (button_pressed(*input, 4)) report[4] |= 0x40;  // R
    if (button_pressed(*input, 5)) report[4] |= 0x80;  // ZR

    if (button_pressed(*input, 14)) report[5] |= 0x01; // Minus
    if (button_pressed(*input, 6)) report[5] |= 0x02;  // Plus
    if (button_pressed(*input, 7)) report[5] |= 0x04;  // RStick
    if (button_pressed(*input, 15)) report[5] |= 0x08; // LStick
    if (button_pressed(*input, 16)) report[5] |= 0x10; // Home
    if (button_pressed(*input, 17)) report[5] |= 0x20; // Capture
    if (button_pressed(*input, 20)) report[5] |= 0x40; // C

    if (button_pressed(*input, 8)) report[6] |= 0x01;  // DDown
    if (button_pressed(*input, 11)) report[6] |= 0x02; // DUp
    if (button_pressed(*input, 9)) report[6] |= 0x04;  // DRight
    if (button_pressed(*input, 10)) report[6] |= 0x08; // DLeft
    if (button_pressed(*input, 12)) report[6] |= 0x40; // L
    if (button_pressed(*input, 13)) report[6] |= 0x80; // ZL

    if (button_pressed(*input, 18)) report[7] |= 0x01; // GR
    if (button_pressed(*input, 19)) report[7] |= 0x02; // GL
    const StickAxes12 left_stick = normalize_stick_pair(input->lx, input->ly);
    const StickAxes12 right_stick = normalize_stick_pair(input->rx, input->ry);
    pack_stick(report, 10, left_stick.x, left_stick.y);
    pack_stick(report, 13, right_stick.x, right_stick.y);
    if (input->motion_valid && kMotionReportOffset + kMotionSampleSize <= kInputReportPayloadLen) {
        std::memcpy(report + kMotionReportOffset, input->motion, kMotionSampleSize);
    }
    s_usb.parsed_reports++;
    if (s_usb.debug_live_log && s_usb.debug_live_logged < 200) {
        ESP_LOGW(TAG,
                 "live buttons=0x%08lx out=%02x %02x %02x %02x sticks=%u,%u %u,%u",
                 static_cast<unsigned long>(input->buttons),
                 report[4],
                 report[5],
                 report[6],
                 report[7],
                 static_cast<unsigned>(input->lx),
                 static_cast<unsigned>(input->ly),
                 static_cast<unsigned>(input->rx),
                 static_cast<unsigned>(input->ry));
        s_usb.debug_live_logged++;
    }
}

int16_t axis_to_xinput_normalized(uint16_t normalized, bool invert) {
    int32_t centered = static_cast<int32_t>(normalized) - kStickCenter12;
    int32_t scaled = centered >= 0 ?
        (centered * 32767) / 2047 :
        (centered * 32768) / 2048;
    if (invert) {
        scaled = -scaled;
    }
    return static_cast<int16_t>(std::max<int32_t>(-32768, std::min<int32_t>(32767, scaled)));
}

uint8_t axis_to_u8_normalized(uint16_t normalized, bool invert = false) {
    if (invert) {
        normalized = static_cast<uint16_t>(4095 - normalized);
    }
    return static_cast<uint8_t>((static_cast<uint32_t>(std::min<uint16_t>(normalized, 4095)) * 255u + 2047u) / 4095u);
}

void put_le16(uint8_t *out, size_t offset, int16_t value) {
    const uint16_t u = static_cast<uint16_t>(value);
    out[offset] = static_cast<uint8_t>(u & 0xff);
    out[offset + 1] = static_cast<uint8_t>((u >> 8) & 0xff);
}

void fill_xinput_report(const InputSnapshot *input, uint8_t report[kXInputReportLen]) {
    std::memset(report, 0, kXInputReportLen);
    report[0] = 0x00;
    report[1] = kXInputReportLen;
    if (input == nullptr || !input->valid) {
        return;
    }

    if (button_pressed(*input, 11)) report[2] |= 0x01; // Up
    if (button_pressed(*input, 8)) report[2] |= 0x02;  // Down
    if (button_pressed(*input, 10)) report[2] |= 0x04; // Left
    if (button_pressed(*input, 9)) report[2] |= 0x08;  // Right
    if (button_pressed(*input, 6)) report[2] |= 0x10;  // Start
    if (button_pressed(*input, 14)) report[2] |= 0x20; // Back
    if (button_pressed(*input, 15)) report[2] |= 0x40; // LS
    if (button_pressed(*input, 7)) report[2] |= 0x80;  // RS

    if (button_pressed(*input, 12)) report[3] |= 0x01; // LB
    if (button_pressed(*input, 4)) report[3] |= 0x02;  // RB
    if (button_pressed(*input, 16)) report[3] |= 0x04; // Guide
    if (button_pressed(*input, 0)) report[3] |= 0x10;  // A
    if (button_pressed(*input, 1)) report[3] |= 0x20;  // B
    if (button_pressed(*input, 2)) report[3] |= 0x40;  // X
    if (button_pressed(*input, 3)) report[3] |= 0x80;  // Y

    report[4] = button_pressed(*input, 13) ? 255 : 0;
    report[5] = button_pressed(*input, 5) ? 255 : 0;
    const StickAxes12 left_stick = normalize_stick_pair(input->lx, input->ly);
    const StickAxes12 right_stick = normalize_stick_pair(input->rx, input->ry);
    put_le16(report, 6, axis_to_xinput_normalized(left_stick.x, false));
    put_le16(report, 8, axis_to_xinput_normalized(left_stick.y, false));
    put_le16(report, 10, axis_to_xinput_normalized(right_stick.x, false));
    put_le16(report, 12, axis_to_xinput_normalized(right_stick.y, false));
}

void fill_dualsense_report(const InputSnapshot *input, uint8_t report[kDualSenseInputPayloadLen]) {
    std::memset(report, 0, kDualSenseInputPayloadLen);
    report[0] = 128;
    report[1] = 128;
    report[2] = 128;
    report[3] = 128;
    report[4] = 0;
    report[5] = 0;
    report[7] = 0x08;
    if (input == nullptr || !input->valid) {
        return;
    }

    const StickAxes12 left_stick = normalize_stick_pair(input->lx, input->ly);
    const StickAxes12 right_stick = normalize_stick_pair(input->rx, input->ry);
    report[0] = axis_to_u8_normalized(left_stick.x);
    report[1] = axis_to_u8_normalized(left_stick.y, true);
    report[2] = axis_to_u8_normalized(right_stick.x);
    report[3] = axis_to_u8_normalized(right_stick.y, true);
    report[4] = button_pressed(*input, 13) ? 255 : 0;
    report[5] = button_pressed(*input, 5) ? 255 : 0;

    uint8_t hat = 0x08;
    const bool up = button_pressed(*input, 11);
    const bool down = button_pressed(*input, 8);
    const bool left = button_pressed(*input, 10);
    const bool right = button_pressed(*input, 9);
    if (up && right) hat = 1;
    else if (down && right) hat = 3;
    else if (down && left) hat = 5;
    else if (up && left) hat = 7;
    else if (up) hat = 0;
    else if (right) hat = 2;
    else if (down) hat = 4;
    else if (left) hat = 6;
    report[7] = hat;
    if (button_pressed(*input, 2)) report[7] |= 0x10; // Square
    if (button_pressed(*input, 0)) report[7] |= 0x20; // Cross
    if (button_pressed(*input, 1)) report[7] |= 0x40; // Circle
    if (button_pressed(*input, 3)) report[7] |= 0x80; // Triangle
    if (button_pressed(*input, 12)) report[8] |= 0x01; // L1
    if (button_pressed(*input, 4)) report[8] |= 0x02;  // R1
    if (button_pressed(*input, 13)) report[8] |= 0x04; // L2
    if (button_pressed(*input, 5)) report[8] |= 0x08;  // R2
    if (button_pressed(*input, 14)) report[8] |= 0x10; // Create
    if (button_pressed(*input, 6)) report[8] |= 0x20;  // Options
    if (button_pressed(*input, 15)) report[8] |= 0x40; // L3
    if (button_pressed(*input, 7)) report[8] |= 0x80;  // R3
    if (button_pressed(*input, 16)) report[9] |= 0x01; // PS
}

int clamp_int(int value, int min_value, int max_value) {
    return std::max(min_value, std::min(max_value, value));
}

int map_switch_amp_to_ble(int value) {
    const int64_t scaled = static_cast<int64_t>(value) * 1023LL * s_usb.rumble_scale_percent;
    return clamp_int(static_cast<int>((scaled + 1450000LL) / 2900000LL), 0, 1023);
}

void build_ble_vibration_data(uint16_t lf_freq,
                              bool lf_tone,
                              uint16_t lf_amp,
                              uint16_t hf_freq,
                              bool hf_tone,
                              uint16_t hf_amp,
                              uint8_t out[5]) {
    uint64_t value = 0;
    value |= static_cast<uint64_t>(lf_freq & 0x01ff);
    value |= static_cast<uint64_t>(lf_tone ? 1 : 0) << 9;
    value |= static_cast<uint64_t>(lf_amp & 0x03ff) << 10;
    value |= static_cast<uint64_t>(hf_freq & 0x01ff) << 20;
    value |= static_cast<uint64_t>(hf_tone ? 1 : 0) << 29;
    value |= static_cast<uint64_t>(hf_amp & 0x03ff) << 30;
    for (size_t i = 0; i < 5; ++i) {
        out[i] = static_cast<uint8_t>((value >> (8 * i)) & 0xff);
    }
}

void build_zero_ble_vibration(uint8_t out[5]) {
    build_ble_vibration_data(0x0e1, false, 0, 0x1e1, false, 0, out);
}

uint8_t scale_u8_percent(uint8_t value, uint16_t percent) {
    const uint32_t clamped = static_cast<uint32_t>(clamp_int(static_cast<int>(percent), 0, 100));
    return static_cast<uint8_t>((static_cast<uint32_t>(value) * clamped + 50u) / 100u);
}

uint16_t scale_dual_motor_amp(uint8_t value) {
    return static_cast<uint16_t>((static_cast<uint32_t>(value) * kDualMotorMaxAmplitude + 127u) / 255u);
}

uint16_t mix_frequency(uint16_t low, uint16_t high, uint8_t value) {
    return static_cast<uint16_t>(low + (((static_cast<uint32_t>(high - low) * value) + 127u) / 255u));
}

uint16_t clamp_ds5_frequency(int value) {
    return static_cast<uint16_t>(clamp_int(value, 0, 0x3ff));
}

float current_ds5_rumble_gain() {
    return static_cast<float>(clamp_int(static_cast<int>(s_usb.rumble_scale_percent), 0, 200)) / 100.0f;
}

uint16_t map_ds5_frequency(uint8_t strength,
                           uint16_t min_value,
                           uint16_t max_value,
                           bool audio_haptics_style,
                           bool high_band) {
    if (strength == 0) {
        return static_cast<uint16_t>((min_value + max_value) / 2);
    }

    const float t = static_cast<float>(strength) / 255.0f;
    if (audio_haptics_style) {
        const uint16_t lifted_min = high_band ? 0x184 : 0x118;
        const uint16_t lifted_max = high_band ? 0x1b0 : 0x148;
        const float shaped_audio = std::pow(t, 0.52f);
        const float lifted_value =
            static_cast<float>(lifted_min) +
            (static_cast<float>(lifted_max - lifted_min) * shaped_audio);
        return clamp_ds5_frequency(static_cast<int>(std::lround(lifted_value)));
    }

    const float value = static_cast<float>(min_value) + (static_cast<float>(max_value - min_value) * t);
    return clamp_ds5_frequency(static_cast<int>(std::lround(value)));
}

uint16_t map_ds5_amplitude(uint8_t strength, float scale = 1.0f) {
    const float amplitude = static_cast<float>(strength) *
        kDs5OutputStrengthScale *
        current_ds5_rumble_gain() *
        scale;
    return static_cast<uint16_t>(clamp_int(static_cast<int>(std::lround(amplitude)), 0, 0x3ff));
}

void pack_rumble_frame(uint16_t high_amp,
                       uint16_t low_amp,
                       uint16_t high_freq,
                       uint16_t low_freq,
                       uint8_t out[5]) {
    build_ble_vibration_data(low_freq, false, low_amp, high_freq, false, high_amp, out);
}

uint16_t map_ds5_spectral_amplitude(uint16_t rms) {
    if (rms < kDs5HapticsMinRms) {
        return 0;
    }
    const float normalized = static_cast<float>(rms) / 32768.0f;
    const float amplitude = normalized * 1023.0f * kDs5SpectralAmplitudeGain * current_ds5_rumble_gain();
    return static_cast<uint16_t>(clamp_int(static_cast<int>(std::lround(amplitude)), 0, 0x3ff));
}

void build_ds5_spectral_frames(uint16_t low_freq,
                               uint16_t high_freq,
                               uint16_t low_rms,
                               uint16_t high_rms,
                               uint8_t out[3][5]) {
    uint16_t low_amp = map_ds5_spectral_amplitude(low_rms);
    uint16_t high_amp = map_ds5_spectral_amplitude(high_rms);

    // Reject cross-band leakage so a pure tone does not drive both actuators.
    if (low_amp < high_amp / 4u) {
        low_amp = 0;
    }
    if (high_amp < low_amp / 4u) {
        high_amp = 0;
    }

    low_freq = static_cast<uint16_t>(clamp_int(low_freq, 70, 300));
    high_freq = static_cast<uint16_t>(clamp_int(high_freq, 250, 511));
    for (size_t i = 0; i < 3; ++i) {
        pack_rumble_frame(high_amp, low_amp, high_freq, low_freq, out[i]);
    }
}

void build_ds5_rumble_frames(uint8_t strong, uint8_t weak, bool audio_haptics_style, uint8_t out[3][5]) {
    struct Profile {
        float high_scale;
        float low_scale;
        int high_delta;
        int low_delta;
    };
    static constexpr Profile kAudioProfiles[3] = {
        {1.08f, 0.96f, +0x0c, +0x05},
        {0.64f, 0.58f, +0x02, -0x01},
        {0.32f, 0.28f, -0x09, -0x07},
    };

    const uint16_t high_freq = map_ds5_frequency(
        weak,
        kDs5HighFrequencyMin,
        kDs5HighFrequencyMax,
        audio_haptics_style,
        true);
    const uint16_t low_freq = map_ds5_frequency(
        strong,
        kDs5LowFrequencyMin,
        kDs5LowFrequencyMax,
        audio_haptics_style,
        false);

    if (!audio_haptics_style) {
        const uint16_t high_amp = map_ds5_amplitude(weak, 1.0f);
        const uint16_t low_amp = map_ds5_amplitude(strong, 1.0f);
        for (size_t i = 0; i < 3; ++i) {
            pack_rumble_frame(high_amp, low_amp, high_freq, low_freq, out[i]);
        }
        return;
    }

    for (size_t i = 0; i < 3; ++i) {
        const Profile &entry = kAudioProfiles[i];
        const uint16_t high_amp = map_ds5_amplitude(
            weak,
            entry.high_scale * kDs5AudioHapticsBaseStrengthScale);
        const uint16_t low_amp = map_ds5_amplitude(
            strong,
            entry.low_scale * kDs5AudioHapticsBaseStrengthScale);
        pack_rumble_frame(
            high_amp,
            low_amp,
            clamp_ds5_frequency(static_cast<int>(high_freq) + entry.high_delta),
            clamp_ds5_frequency(static_cast<int>(low_freq) + entry.low_delta),
            out[i]);
    }
}

void build_ble_vibration_from_dual_motor(uint8_t weak, uint8_t strong, uint8_t out[5]) {
    const uint16_t low_amp = scale_dual_motor_amp(strong);
    const uint16_t high_amp = scale_dual_motor_amp(weak);
    const uint16_t low_freq = low_amp == 0 ? 0x0e1 : mix_frequency(0x0b8, 0x122, strong);
    const uint16_t high_freq = high_amp == 0 ? 0x1e1 : mix_frequency(0x160, 0x1f0, weak);
    build_ble_vibration_data(low_freq, false, low_amp, high_freq, false, high_amp, out);
}

void encode_ble_vibration_from_switch_frame(const uint8_t *report, uint16_t len, uint16_t offset, uint8_t out[5]) {
    if (len < offset + 5) {
        build_zero_ble_vibration(out);
        return;
    }
    const int b0 = report[offset];
    const int b1 = report[offset + 1];
    const int b2 = report[offset + 2];
    const int b3 = report[offset + 3];
    const int b4 = report[offset + 4];
    const int high_freq = b0 | ((b1 & 0x03) << 8);
    const int high_amp = ((b1 & 0xfc) << 4) | ((b2 & 0x0f) << 12);
    const int low_freq = ((b2 & 0xf0) >> 4) | ((b3 & 0x3f) << 4);
    const int low_amp = (b3 & 0xc0) | (b4 << 8);
    build_ble_vibration_data(static_cast<uint16_t>(low_freq),
                             false,
                             static_cast<uint16_t>(map_switch_amp_to_ble(low_amp)),
                             static_cast<uint16_t>(high_freq),
                             false,
                             static_cast<uint16_t>(map_switch_amp_to_ble(high_amp)),
                             out);
}

void write_motor_block(uint8_t *out, uint16_t offset, uint8_t packet_id, const uint8_t frames[3][5]) {
    out[offset] = static_cast<uint8_t>(0x50 | (packet_id & 0x0f));
    std::memcpy(out + offset + 1, frames[0], 5);
    std::memcpy(out + offset + 6, frames[1], 5);
    std::memcpy(out + offset + 11, frames[2], 5);
}

void build_pro2_hd_packet(uint8_t packet_id, const uint8_t left[3][5], const uint8_t right[3][5], uint8_t out[33]) {
    std::memset(out, 0, 33);
    out[0] = 0x00;
    write_motor_block(out, 1, packet_id, left);
    write_motor_block(out, 17, packet_id, right);
}

bool has_non_zero_payload(const uint8_t *data, uint16_t len, uint16_t offset) {
    for (uint16_t i = offset; i < len; ++i) {
        if (data[i] != 0) {
            return true;
        }
    }
    return false;
}

bool has_neutral_rumble_frame(const uint8_t *data, uint16_t len, uint16_t offset) {
    return len >= offset + 5 &&
           data[offset] == 0x87 &&
           data[offset + 1] == 0x01 &&
           data[offset + 2] == 0x20 &&
           data[offset + 3] == 0x11 &&
           data[offset + 4] == 0x00;
}

bool is_switch2_hid_rumble_report(const uint8_t *data, uint16_t len) {
    return len >= 7 && data[0] == kUsbReportIdOutput && (data[1] & 0xf0) == 0x50;
}

bool is_neutral_switch_rumble(const uint8_t *data, uint16_t len) {
    return has_neutral_rumble_frame(data, len, 2) &&
           has_neutral_rumble_frame(data, len, 0x12);
}

void fill_rumble_frames(const uint8_t frame[5], uint8_t out[3][5]) {
    for (size_t i = 0; i < 3; ++i) {
        std::memcpy(out[i], frame, 5);
    }
}

void fill_zero_rumble_frames(uint8_t out[3][5]) {
    uint8_t zero[5];
    build_zero_ble_vibration(zero);
    fill_rumble_frames(zero, out);
}

void stop_rumble() {
    fill_zero_rumble_frames(s_usb.rumble_left);
    fill_zero_rumble_frames(s_usb.rumble_right);
    s_usb.rumble_until_us = 0;
    s_usb.rumble_active = false;
    s_usb.rumble_stop_pending = s_usb.rumble_stop_packets;
    s_usb.rumble_next_tick_us = 0;
    s_usb.rumble_stops++;
}

void update_rumble_stream_frames(const uint8_t left[3][5], const uint8_t right[3][5]) {
    if (!s_usb.rumble_enabled) {
        stop_rumble();
        return;
    }
    const bool was_active = s_usb.rumble_active;
    std::memcpy(s_usb.rumble_left, left, sizeof(s_usb.rumble_left));
    std::memcpy(s_usb.rumble_right, right, sizeof(s_usb.rumble_right));
    s_usb.rumble_until_us = esp_timer_get_time() + static_cast<uint64_t>(s_usb.rumble_hold_ms) * 1000ULL;
    s_usb.rumble_active = true;
    if (!was_active) {
        s_usb.rumble_next_tick_us = 0;
        s_usb.rumble_stream_logged = 0;
    }
    s_usb.rumble_updates++;
}

void update_rumble_stream(const uint8_t left[5], const uint8_t right[5]) {
    uint8_t left_frames[3][5];
    uint8_t right_frames[3][5];
    fill_rumble_frames(left, left_frames);
    fill_rumble_frames(right, right_frames);
    update_rumble_stream_frames(left_frames, right_frames);
}

void remember_dual_motor_rumble(const char *source, uint8_t left_heavy, uint8_t right_light) {
    s_usb.rumble_last_left_heavy = left_heavy;
    s_usb.rumble_last_right_light = right_light;
    std::snprintf(s_usb.rumble_last_source, sizeof(s_usb.rumble_last_source), "%s", source != nullptr ? source : "dual");
}

uint8_t peak_u8(uint8_t a, uint8_t b) {
    return a > b ? a : b;
}

void reset_ds5_haptics_state() {
    s_usb.ds5_haptic_right_low = 0;
    s_usb.ds5_haptic_right_high = 0;
    s_usb.ds5_haptic_left_low = 0;
    s_usb.ds5_haptic_left_high = 0;
    std::memset(s_usb.ds5_haptics_left_samples, 0, sizeof(s_usb.ds5_haptics_left_samples));
    std::memset(s_usb.ds5_haptics_right_samples, 0, sizeof(s_usb.ds5_haptics_right_samples));
    s_usb.ds5_haptics_sample_pos = 0;
    s_usb.ds5_haptic_left_low_freq = 0x112;
    s_usb.ds5_haptic_left_high_freq = 0x187;
    s_usb.ds5_haptic_right_low_freq = 0x112;
    s_usb.ds5_haptic_right_high_freq = 0x187;
    s_usb.ds5_haptic_left_low_rms = 0;
    s_usb.ds5_haptic_left_high_rms = 0;
    s_usb.ds5_haptic_right_low_rms = 0;
    s_usb.ds5_haptic_right_high_rms = 0;
    s_usb.ds5_audio_decim_count = 0;
    s_usb.ds5_audio_left_sum = 0;
    s_usb.ds5_audio_right_sum = 0;
}

void update_ds5_rumble_mix(const char *source, uint16_t hold_ms) {
    const uint8_t regular_right_low = s_usb.ds5_regular_right;
    const uint8_t regular_right_high = s_usb.ds5_regular_right;
    const uint8_t regular_left_low = s_usb.ds5_regular_left;
    const uint8_t regular_left_high = s_usb.ds5_regular_left;

    const uint8_t right_low = peak_u8(regular_right_low, s_usb.ds5_haptic_right_low);
    const uint8_t right_high = peak_u8(regular_right_high, s_usb.ds5_haptic_right_high);
    const uint8_t left_low = peak_u8(regular_left_low, s_usb.ds5_haptic_left_low);
    const uint8_t left_high = peak_u8(regular_left_high, s_usb.ds5_haptic_left_high);

    const bool has_rumble = s_usb.ds5_regular_right != 0 || s_usb.ds5_regular_left != 0;
    const bool has_haptics =
        s_usb.ds5_haptic_right_low != 0 || s_usb.ds5_haptic_right_high != 0 ||
        s_usb.ds5_haptic_left_low != 0 || s_usb.ds5_haptic_left_high != 0;
    if (!has_rumble && !has_haptics) {
        stop_rumble();
        remember_dual_motor_rumble(source, 0, 0);
        return;
    }

    uint8_t left_frames[3][5];
    uint8_t right_frames[3][5];
    const bool has_left_haptics = s_usb.ds5_haptic_left_low != 0 || s_usb.ds5_haptic_left_high != 0;
    const bool has_right_haptics = s_usb.ds5_haptic_right_low != 0 || s_usb.ds5_haptic_right_high != 0;
    if (has_left_haptics) {
        build_ds5_spectral_frames(
            s_usb.ds5_haptic_left_low_freq,
            s_usb.ds5_haptic_left_high_freq,
            s_usb.ds5_haptic_left_low_rms,
            s_usb.ds5_haptic_left_high_rms,
            left_frames);
    } else {
        build_ds5_rumble_frames(left_low, left_high, false, left_frames);
    }
    if (has_right_haptics) {
        build_ds5_spectral_frames(
            s_usb.ds5_haptic_right_low_freq,
            s_usb.ds5_haptic_right_high_freq,
            s_usb.ds5_haptic_right_low_rms,
            s_usb.ds5_haptic_right_high_rms,
            right_frames);
    } else {
        build_ds5_rumble_frames(right_low, right_high, false, right_frames);
    }
    update_rumble_stream_frames(left_frames, right_frames);
    s_usb.rumble_until_us = esp_timer_get_time() + static_cast<uint64_t>(hold_ms) * 1000ULL;
    remember_dual_motor_rumble(source, left_low, right_high);
}

void bridge_dual_motor_output_to_ble(const char *source,
                                     uint8_t left_heavy,
                                     uint8_t right_light,
                                     uint16_t hold_ms) {
    remember_dual_motor_rumble(source, left_heavy, right_light);
    s_usb.rumble_dual_motor_reports++;
    if (left_heavy == 0 && right_light == 0) {
        stop_rumble();
        return;
    }

    uint8_t left[5];
    uint8_t right[5];
    const uint8_t weak = scale_u8_percent(right_light, s_usb.rumble_scale_percent);
    const uint8_t strong = scale_u8_percent(left_heavy, s_usb.rumble_scale_percent);
    build_ble_vibration_from_dual_motor(weak, strong, left);
    build_ble_vibration_from_dual_motor(weak, strong, right);
    update_rumble_stream(left, right);
    s_usb.rumble_until_us = esp_timer_get_time() + static_cast<uint64_t>(hold_ms) * 1000ULL;
}

bool bridge_ds5_output_to_ble(uint8_t report_id, const uint8_t *buffer, uint16_t len) {
    if (s_usb.output_mode != OutputMode::DualSense) {
        return false;
    }
    if (buffer == nullptr || len == 0 || (report_id != 0 && report_id != kDs5OutputReportId)) {
        return false;
    }

    const uint8_t *payload = buffer;
    uint16_t payload_len = len;
    if (report_id == 0) {
        if (buffer[0] != kDs5OutputReportId) {
            return false;
        }
        payload = buffer + 1;
        payload_len = static_cast<uint16_t>(payload_len - 1);
    }
    if (payload_len < 4) {
        return false;
    }

    bool enabled = (payload[0] & kDs5RumbleEnableMask) != 0;
    if (payload_len > kDs5ImprovedRumbleOffset) {
        enabled = enabled || ((payload[kDs5ImprovedRumbleOffset] & kDs5ImprovedRumbleMask) != 0);
    }

    const uint8_t right_light = payload[2];
    const uint8_t left_heavy = payload[3];
    s_usb.rumble_ds5_reports++;
    if (!enabled) {
        s_usb.ds5_regular_right = 0;
        s_usb.ds5_regular_left = 0;
        update_ds5_rumble_mix((right_light == 0 && left_heavy == 0) ? "ds5" : "ds5_ignored", kHostRumbleHoldMs);
        return true;
    }

    s_usb.ds5_regular_right = right_light;
    s_usb.ds5_regular_left = left_heavy;
    update_ds5_rumble_mix("ds5", kHostRumbleHoldMs);
    return true;
}

bool parse_xinput_rumble_out(const uint8_t *data, uint16_t len, uint8_t *left_heavy, uint8_t *right_light) {
    if (data == nullptr || left_heavy == nullptr || right_light == nullptr || len < 5) {
        return false;
    }

    if (data[0] != 0x00 || data[1] != 0x08) {
        return false;
    }

    *left_heavy = data[3];
    *right_light = data[4];
    return true;
}

bool bridge_xinput_output_to_ble(const uint8_t *data, uint16_t len) {
    if (s_usb.output_mode != OutputMode::XInput) {
        return false;
    }
    uint8_t left_heavy = 0;
    uint8_t right_light = 0;
    if (!parse_xinput_rumble_out(data, len, &left_heavy, &right_light)) {
        return false;
    }

    s_usb.rumble_xinput_reports++;
    if (s_usb.rumble_xinput_out_logged < 16) {
        const uint8_t b0 = len > 0 ? data[0] : 0;
        const uint8_t b1 = len > 1 ? data[1] : 0;
        const uint8_t b2 = len > 2 ? data[2] : 0;
        const uint8_t b3 = len > 3 ? data[3] : 0;
        const uint8_t b4 = len > 4 ? data[4] : 0;
        const uint8_t b5 = len > 5 ? data[5] : 0;
        const uint8_t b6 = len > 6 ? data[6] : 0;
        const uint8_t b7 = len > 7 ? data[7] : 0;
        ESP_LOGW(TAG,
                 "xinput out len=%u data=%02x %02x %02x %02x %02x %02x %02x %02x motor=%u/%u",
                 static_cast<unsigned>(len),
                 b0,
                 b1,
                 b2,
                 b3,
                 b4,
                 b5,
                 b6,
                 b7,
                 static_cast<unsigned>(left_heavy),
                 static_cast<unsigned>(right_light));
        s_usb.rumble_xinput_out_logged++;
    }
    bridge_dual_motor_output_to_ble("xinput", left_heavy, right_light, kHostRumbleHoldMs);
    return true;
}

bool bridge_hid_output_to_ble(const uint8_t *data, uint16_t len) {
    if (s_usb.output_mode != OutputMode::Nintendo) {
        return false;
    }
    if (data == nullptr || len < 2 || !is_switch2_hid_rumble_report(data, len)) {
        return false;
    }
    s_usb.rumble_switch_reports++;
    std::snprintf(s_usb.rumble_last_source, sizeof(s_usb.rumble_last_source), "%s", "switch");
    const bool active = has_non_zero_payload(data, len, 2) && !is_neutral_switch_rumble(data, len);
    if (!active) {
        stop_rumble();
        return true;
    }
    uint8_t left[5];
    uint8_t right[5];
    encode_ble_vibration_from_switch_frame(data, len, 2, left);
    encode_ble_vibration_from_switch_frame(data, len, 0x12, right);
    update_rumble_stream(left, right);
    return true;
}

bool hid_guard_timed_out() {
    return s_usb.hid_guard_active &&
           (esp_timer_get_time() - s_usb.hid_guard_started_us) > kHidGuardTimeoutUs;
}

void release_hid_guard(const char *reason) {
    if (s_usb.hid_guard_active || s_usb.hid_guard_release_after_tx) {
        ESP_LOGW(TAG, "Steam init guard released: %s", reason != nullptr ? reason : "done");
    }
    s_usb.hid_guard_active = false;
    s_usb.hid_guard_done = true;
    s_usb.hid_guard_release_after_tx = false;
}

bool hid_guard_active() {
    if (hid_guard_timed_out()) {
        release_hid_guard("timeout");
    }
    return s_usb.hid_guard_active;
}

size_t flash_read_length(uint32_t address) {
    if (address == 0x13040) {
        return 0x10;
    }
    if (address == 0x13100) {
        return 0x18;
    }
    if (address == 0x13060) {
        return 0x20;
    }
    return 0x40;
}

size_t build_vendor_ack(const uint8_t *cmd, uint16_t cmd_len, uint8_t *reply, size_t reply_len) {
    if (cmd == nullptr || reply == nullptr || reply_len == 0) {
        return 0;
    }
    std::memset(reply, 0, reply_len);
    reply[0] = cmd[0];
    if (reply_len > 1) {
        reply[1] = 0x01;
    }
    if (reply_len > 2 && cmd_len > 2) {
        reply[2] = cmd[2];
    }
    if (reply_len > 3 && cmd_len > 3) {
        reply[3] = cmd[3];
    }
    if (reply_len > 4 && cmd_len > 4) {
        reply[4] = cmd[4];
    }
    if (reply_len > 5) {
        reply[5] = 0xf8;
    }
    return reply_len;
}

size_t build_flash_read_reply(const uint8_t *cmd, uint16_t cmd_len, uint8_t *reply, size_t reply_max) {
    if (cmd == nullptr || reply == nullptr || cmd_len < 16) {
        return 0;
    }

    const uint32_t address = command_address(cmd, cmd_len);
    const size_t data_len = flash_read_length(address);
    size_t full_reply_len = 0x10 + data_len;
    if (full_reply_len > reply_max) {
        return 0;
    }

    std::memset(reply, 0, full_reply_len);
    uint8_t *data = reply + 0x10;

    if (address == 0x13000) {
        static const uint8_t serial[] = {'H', 'A', '2', 'F', '8', '3', 'J', 'F'};
        write_bytes(data, data_len, 2, serial, sizeof(serial));
    }

    if (address == 0x13080 || address == 0x130c0) {
        std::memset(data, 0xff, data_len);
        uint8_t calib[9];
        pack_stick_calibration(calib);
        write_bytes(data, data_len, 0x28, calib, sizeof(calib));
    }

    if (address == 0x1fc040 || address == 0x1fc080 || address == 0x13060) {
        std::memset(data, 0xff, data_len);
    }

    if (address == 0x13040) {
        static const uint8_t block[] = {
            0x16, 0xf4, 0xd3, 0x41, 0x48, 0xce, 0x85, 0xba,
            0xf1, 0x05, 0x71, 0xba, 0x1f, 0x27, 0xcb, 0x3b,
        };
        write_bytes(data, data_len, 0, block, sizeof(block));
    }

    if (address == 0x13100) {
        static const uint8_t block[] = {
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x2d, 0x10, 0xa7, 0x3d,
            0xe7, 0x49, 0x35, 0x3c, 0xa4, 0x2d, 0x20, 0x41,
        };
        write_bytes(data, data_len, 0, block, sizeof(block));
    }

    reply[0] = 0x02;
    reply[1] = 0x01;
    reply[2] = cmd[2];
    reply[3] = cmd[3];
    reply[5] = 0xf8;
    reply[8] = static_cast<uint8_t>(data_len);
    std::memcpy(reply + 12, cmd + 12, 4);
    if (full_reply_len > kFlashReplyFullSpeedLen) {
        full_reply_len = kFlashReplyFullSpeedLen;
    }
    return full_reply_len;
}

size_t build_vendor_reply(const uint8_t *cmd, uint16_t cmd_len, uint8_t *reply, size_t reply_max) {
    if (cmd == nullptr || cmd_len == 0 || reply == nullptr || reply_max == 0) {
        return 0;
    }

    const uint8_t command = cmd[0];
    const uint8_t arg = cmd_len > 3 ? cmd[3] : 0;

    if (cmd_len >= 16 && command == 0x02) {
        return build_flash_read_reply(cmd, cmd_len, reply, reply_max);
    }
    if (command == 0x0c && arg == 0x02) {
        return 0;
    }
    if (command == 0x10) {
        return 0;
    }
    if (command == 0x03 && arg == 0x0d) {
        const size_t n = build_vendor_ack(cmd, cmd_len, reply, 12);
        reply[8] = 0x01;
        return n;
    }
    if (command == 0x15 && arg == 0x01) {
        static const uint8_t mac_le[] = {0x2d, 0xfc, 0x27, 0xce, 0xc6, 0x38};
        const size_t n = build_vendor_ack(cmd, cmd_len, reply, 17);
        reply[8] = 0x01;
        reply[9] = 0x04;
        reply[10] = 0x01;
        write_bytes(reply, n, 11, mac_le, sizeof(mac_le));
        return n;
    }
    if (command == 0x15 && arg == 0x02) {
        const size_t n = build_vendor_ack(cmd, cmd_len, reply, 25);
        reply[8] = 0x01;
        return n;
    }
    if (command == 0x15 && arg == 0x03) {
        const size_t n = build_vendor_ack(cmd, cmd_len, reply, 9);
        reply[8] = 0x01;
        return n;
    }
    if (command == 0x11) {
        static const uint8_t payload[] = {
            0x20, 0x03, 0x00, 0x00, 0x0a, 0xe8, 0x1c, 0x3b,
            0x79, 0x7d, 0x8b, 0x3a, 0x0a, 0xe8, 0x9c, 0x42,
            0x58, 0xa0, 0x0b, 0x42, 0x0a, 0xe8, 0x9c, 0x41,
            0x58, 0xa0, 0x0b, 0x41,
        };
        const size_t n = build_vendor_ack(cmd, cmd_len, reply, 37);
        reply[8] = 0x01;
        write_bytes(reply, n, 9, payload, sizeof(payload));
        return n;
    }
    if (command == 0x01 && arg == 0x0c) {
        static const uint8_t payload[] = {0x61, 0x12, 0x50, 0x10};
        const size_t n = build_vendor_ack(cmd, cmd_len, reply, 12);
        write_bytes(reply, n, 8, payload, sizeof(payload));
        return n;
    }
    if (command == 0x03 && arg == 0x01) {
        const size_t n = build_vendor_ack(cmd, cmd_len, reply, 16);
        reply[10] = 0x40;
        reply[11] = 0xf0;
        reply[14] = 0x60;
        return n;
    }
    return build_vendor_ack(cmd, cmd_len, reply, 8);
}

void flush_vendor_pending() {
    if (s_usb.vendor_pending_len == 0 || s_usb.vendor_pending_offset >= s_usb.vendor_pending_len) {
        s_usb.vendor_pending_len = 0;
        s_usb.vendor_pending_offset = 0;
        return;
    }

    while (s_usb.vendor_pending_offset < s_usb.vendor_pending_len) {
        const uint32_t available = tud_vendor_n_write_available(s_usb.vendor_pending_itf);
        if (available == 0) {
            break;
        }
        const uint16_t remaining = static_cast<uint16_t>(s_usb.vendor_pending_len - s_usb.vendor_pending_offset);
        const uint32_t chunk = std::min<uint32_t>(available, remaining);
        const uint32_t written = tud_vendor_n_write(s_usb.vendor_pending_itf,
                                                    s_usb.vendor_pending + s_usb.vendor_pending_offset,
                                                    chunk);
        if (written == 0) {
            break;
        }
        s_usb.vendor_pending_offset = static_cast<uint16_t>(s_usb.vendor_pending_offset + written);
    }

    tud_vendor_n_write_flush(s_usb.vendor_pending_itf);
    if (s_usb.vendor_pending_offset >= s_usb.vendor_pending_len) {
        s_usb.vendor_pending_len = 0;
        s_usb.vendor_pending_offset = 0;
    }
}

void queue_vendor_reply(uint8_t itf, const uint8_t *reply, size_t reply_len) {
    if (reply == nullptr || reply_len == 0 || reply_len > sizeof(s_usb.vendor_pending)) {
        return;
    }
    std::memcpy(s_usb.vendor_pending, reply, reply_len);
    s_usb.vendor_pending_len = static_cast<uint16_t>(reply_len);
    s_usb.vendor_pending_offset = 0;
    s_usb.vendor_pending_itf = itf;
    flush_vendor_pending();
}

void start_test_rumble(bool left_on, bool right_on, uint16_t hold_ms, uint16_t amp) {
    uint8_t left[5];
    uint8_t right[5];
    build_zero_ble_vibration(left);
    build_zero_ble_vibration(right);

    const uint16_t clamped_amp = static_cast<uint16_t>(clamp_int(static_cast<int>(amp), 0, 1023));
    if (left_on) {
        build_ble_vibration_data(0x0e1, false, clamped_amp, 0x1e1, false, clamped_amp, left);
    }
    if (right_on) {
        build_ble_vibration_data(0x0e1, false, clamped_amp, 0x1e1, false, clamped_amp, right);
    }

    s_usb.rumble_enabled = true;
    fill_rumble_frames(left, s_usb.rumble_left);
    fill_rumble_frames(right, s_usb.rumble_right);
    s_usb.rumble_until_us = esp_timer_get_time() + static_cast<uint64_t>(hold_ms) * 1000ULL;
    s_usb.rumble_active = true;
    s_usb.rumble_next_tick_us = 0;
    s_usb.rumble_updates++;
}

void inject_mode_rumble(uint8_t left_heavy, uint8_t right_light, uint16_t hold_ms) {
    s_usb.rumble_enabled = true;
    switch (s_usb.output_mode) {
    case OutputMode::DualSense: {
        uint8_t payload[64] = {};
        payload[0] = kDs5RumbleEnableMask;
        payload[2] = right_light;
        payload[3] = left_heavy;
        if (sizeof(payload) > kDs5ImprovedRumbleOffset) {
            payload[kDs5ImprovedRumbleOffset] = kDs5ImprovedRumbleMask;
        }
        bridge_ds5_output_to_ble(kDs5OutputReportId, payload, sizeof(payload));
        s_usb.rumble_until_us = esp_timer_get_time() + static_cast<uint64_t>(hold_ms) * 1000ULL;
        break;
    }
    case OutputMode::XInput: {
        const uint8_t packet[5] = {
            0x00,
            0x08,
            0x00,
            left_heavy,
            right_light,
        };
        bridge_xinput_output_to_ble(packet, sizeof(packet));
        s_usb.rumble_until_us = esp_timer_get_time() + static_cast<uint64_t>(hold_ms) * 1000ULL;
        break;
    }
    case OutputMode::Nintendo:
    default:
        bridge_dual_motor_output_to_ble("manual_nintendo", left_heavy, right_light, hold_ms);
        break;
    }
}

void inject_mode_rumble_test(bool left_on, bool right_on, uint16_t hold_ms, uint16_t amp) {
    const uint8_t heavy = left_on ?
        static_cast<uint8_t>((static_cast<uint32_t>(clamp_int(static_cast<int>(amp), 0, 1023)) * 255u + 511u) / 1023u) :
        0;
    const uint8_t light = right_on ?
        static_cast<uint8_t>((static_cast<uint32_t>(clamp_int(static_cast<int>(amp), 0, 1023)) * 255u + 511u) / 1023u) :
        0;

    if (s_usb.output_mode == OutputMode::Nintendo) {
        start_test_rumble(left_on, right_on, hold_ms, amp);
        std::snprintf(s_usb.rumble_last_source, sizeof(s_usb.rumble_last_source), "%s", "manual_nintendo");
        s_usb.rumble_last_left_heavy = heavy;
        s_usb.rumble_last_right_light = light;
        return;
    }
    inject_mode_rumble(heavy, light, hold_ms);
}

void rumble_task() {
    const uint64_t now = esp_timer_get_time();
    if (s_usb.rumble_next_tick_us != 0 && now < s_usb.rumble_next_tick_us) {
        return;
    }

    uint8_t left[3][5];
    uint8_t right[3][5];
    const bool active = s_usb.rumble_active && now <= s_usb.rumble_until_us;
    if (s_usb.rumble_active && !active) {
        s_usb.rumble_active = false;
        s_usb.rumble_stop_pending = s_usb.rumble_stop_packets;
        s_usb.rumble_stops++;
    }
    if (active) {
        std::memcpy(left, s_usb.rumble_left, sizeof(left));
        std::memcpy(right, s_usb.rumble_right, sizeof(right));
    } else if (s_usb.rumble_stop_pending > 0) {
        s_usb.rumble_stop_pending--;
        fill_zero_rumble_frames(left);
        fill_zero_rumble_frames(right);
    } else {
        return;
    }

    uint8_t packet[33];
    build_pro2_hd_packet(s_usb.rumble_packet_id++ & 0x0f, left, right, packet);
    const bool sent = ble_send_rumble(packet, sizeof(packet));
    if (sent) {
        s_usb.rumble_writes++;
    } else {
        s_usb.rumble_errors++;
    }
    if (s_usb.rumble_stream_logged < 16) {
        ESP_LOGW(TAG,
                 "rumble stream source=%s active=%u stop=%u sent=%u id=%u tick=%u left=%02x %02x %02x %02x %02x right=%02x %02x %02x %02x %02x",
                 s_usb.rumble_last_source,
                 active ? 1u : 0u,
                 static_cast<unsigned>(s_usb.rumble_stop_pending),
                 sent ? 1u : 0u,
                 static_cast<unsigned>((s_usb.rumble_packet_id - 1) & 0x0f),
                 static_cast<unsigned>(s_usb.rumble_tick_ms),
                 left[0][0],
                 left[0][1],
                 left[0][2],
                 left[0][3],
                 left[0][4],
                 right[0][0],
                 right[0][1],
                 right[0][2],
                 right[0][3],
                 right[0][4]);
        s_usb.rumble_stream_logged++;
    }
    s_usb.rumble_next_tick_us = now + static_cast<uint64_t>(s_usb.rumble_tick_ms) * 1000ULL;
}

void set_report_rate(uint32_t rate_hz) {
    const uint32_t requested_rate_hz = rate_hz;
    rate_hz = std::max<uint32_t>(kMinReportRateHz, std::min<uint32_t>(kMaxReportRateHz, rate_hz));
    s_usb.report_rate_hz = static_cast<uint16_t>(rate_hz);
    s_usb.report_interval_us = 1000000u / rate_hz;
    const uint64_t now = esp_timer_get_time();
    s_usb.next_report_us = now + s_usb.report_interval_us;
    s_usb.report_rate_sample_us = now;
    s_usb.report_rate_sample_count = s_usb.reports_sent;
    s_usb.report_submit_hz = 0;
    s_usb.report_last_gap_us = 0;
    s_usb.report_max_gap_us = 0;
    s_usb.report_last_sent_us = 0;
    s_usb.last_reported_input_updates = 0;
    s_usb.report_complete_sample_us = now;
    s_usb.report_complete_sample_count = s_usb.reports_completed;
    s_usb.report_complete_hz = 0;
    s_usb.report_complete_last_gap_us = 0;
    s_usb.report_complete_max_gap_us = 0;
    s_usb.report_last_complete_us = 0;
    s_usb.feature_quiet_until_us = 0;
    ESP_LOGW(TAG, "report rate set requested=%lu target=%u interval_us=%lu",
             static_cast<unsigned long>(requested_rate_hz),
             static_cast<unsigned>(s_usb.report_rate_hz),
             static_cast<unsigned long>(s_usb.report_interval_us));
}

void queue_feature_json(const char *json) {
    if (json == nullptr) {
        json = "{\"ok\":false,\"profile\":\"ns2pro\",\"error\":\"empty_reply\"}";
    }

    size_t len = std::strlen(json);
    if (len > sizeof(s_usb.feature_reply)) {
        json = "{\"ok\":false,\"profile\":\"ns2pro\",\"error\":\"reply_too_large\"}";
        len = std::strlen(json);
    }

    std::memcpy(s_usb.feature_reply, json, len);
    s_usb.feature_reply_len = static_cast<uint16_t>(len);
    s_usb.feature_reply_offset = 0;
}

void queue_feature_error(const char *error) {
    char json[128];
    std::snprintf(json,
                  sizeof(json),
                  "{\"ok\":false,\"profile\":\"ns2pro\",\"platform\":\"esp32s3\",\"error\":\"%s\"}",
                  error);
    queue_feature_json(json);
}

void hold_feature_quiet(uint64_t hold_us = kFeatureQuietUs) {
    const uint64_t until = esp_timer_get_time() + hold_us;
    if (until > s_usb.feature_quiet_until_us) {
        s_usb.feature_quiet_until_us = until;
    }
}

void format_settings_json(char *out, size_t out_len, bool saved = false) {
    std::snprintf(out,
                  out_len,
                  "{\"ok\":true,\"profile\":\"ns2pro\",\"platform\":\"esp32s3\","
                  "\"settings\":true,\"saved\":%s,\"persisted\":true,"
                  "\"output_mode\":\"%s\",\"pending_output_mode\":\"%s\","
                  "\"restart_required\":%s,"
                  "\"usb_raw_passthrough\":%s,\"web_parse_reports\":%s,"
                  "\"rumble_enabled\":%s,"
                  "\"report_rate_hz\":%u,\"rumble_scale_percent\":%u,"
                  "\"rumble_hold_ms\":%u,\"rumble_tick_ms\":%u,"
                  "\"rumble_stop_packets\":%u}",
                  saved ? "true" : "false",
                  config_output_mode_name(s_usb.output_mode),
                  config_output_mode_name(saved_output_mode()),
                  output_mode_restart_required() ? "true" : "false",
                  s_usb.raw_passthrough ? "true" : "false",
                  s_usb.web_parse_reports ? "true" : "false",
                  s_usb.rumble_enabled ? "true" : "false",
                  static_cast<unsigned>(s_usb.report_rate_hz),
                  static_cast<unsigned>(s_usb.rumble_scale_percent),
                  static_cast<unsigned>(s_usb.rumble_hold_ms),
                  static_cast<unsigned>(s_usb.rumble_tick_ms),
                  static_cast<unsigned>(s_usb.rumble_stop_packets));
}

void format_usb_status_json(char *out, size_t out_len) {
    std::snprintf(out,
                  out_len,
                  "{\"ok\":true,\"profile\":\"ns2pro\",\"platform\":\"esp32s3\","
                  "\"ready\":true,\"usb\":\"tinyusb\","
                  "\"output_mode\":\"%s\",\"pending_output_mode\":\"%s\","
                  "\"restart_required\":%s,\"usb_mounted\":%s,"
                  "\"usb_suspended\":%s,\"reports_sent\":%lu,"
                  "\"reports_failed\":%lu,\"raw_passthrough_reports\":%lu,"
                  "\"parsed_reports\":%lu,\"report_rate_hz\":%u,"
                  "\"report_interval_us\":%lu,\"usb_submit_hz\":%lu,"
                  "\"usb_complete_hz\":%lu,\"hid_out\":%lu,"
                  "\"rumble_enabled\":%s,\"rumble_active\":%s,"
                  "\"rumble_writes\":%lu,\"rumble_errors\":%lu,"
                  "\"rumble_scale_percent\":%u,\"rumble_hold_ms\":%u,"
                  "\"rumble_tick_ms\":%u,\"rumble_stop_packets\":%u,"
                  "\"rumble_last_source\":\"%s\","
                  "\"audio_set_interface_count\":%lu,"
                  "\"audio_last_interface\":%u,\"audio_last_alt\":%u,"
                  "\"audio_out_packets\":%lu,\"audio_out_bytes\":%lu,"
                  "\"usb_raw_passthrough\":%s,"
                  "\"web_parse_reports\":%s,\"feature_set\":%lu,\"feature_get\":%lu}",
                  config_output_mode_name(s_usb.output_mode),
                  config_output_mode_name(saved_output_mode()),
                  output_mode_restart_required() ? "true" : "false",
                  s_usb.mounted ? "true" : "false",
                  s_usb.suspended ? "true" : "false",
                  static_cast<unsigned long>(s_usb.reports_sent),
                  static_cast<unsigned long>(s_usb.reports_failed),
                  static_cast<unsigned long>(s_usb.raw_reports),
                  static_cast<unsigned long>(s_usb.parsed_reports),
                  static_cast<unsigned>(s_usb.report_rate_hz),
                  static_cast<unsigned long>(s_usb.report_interval_us),
                  static_cast<unsigned long>(s_usb.report_submit_hz),
                  static_cast<unsigned long>(s_usb.report_complete_hz),
                  static_cast<unsigned long>(s_usb.hid_out_count),
                  s_usb.rumble_enabled ? "true" : "false",
                  s_usb.rumble_active ? "true" : "false",
                  static_cast<unsigned long>(s_usb.rumble_writes),
                  static_cast<unsigned long>(s_usb.rumble_errors),
                  static_cast<unsigned>(s_usb.rumble_scale_percent),
                  static_cast<unsigned>(s_usb.rumble_hold_ms),
                  static_cast<unsigned>(s_usb.rumble_tick_ms),
                  static_cast<unsigned>(s_usb.rumble_stop_packets),
                  s_usb.rumble_last_source,
                  static_cast<unsigned long>(s_usb.audio_set_interface_count),
                  static_cast<unsigned>(s_usb.audio_last_interface),
                  static_cast<unsigned>(s_usb.audio_last_alt),
                  static_cast<unsigned long>(s_usb.audio_out_packets),
                  static_cast<unsigned long>(s_usb.audio_out_bytes),
                  s_usb.raw_passthrough ? "true" : "false",
                  s_usb.web_parse_reports ? "true" : "false",
                  static_cast<unsigned long>(s_usb.feature_set_count),
                  static_cast<unsigned long>(s_usb.feature_get_count));
}

void format_ble_status_json(char *out, size_t out_len) {
    BleStats stats;
    ble_get_stats(&stats);

    char local[18] = {};
    if (stats.local_addr_valid) {
        format_ble_addr(stats.local_addr, local, sizeof(local));
    }
    char saved[18] = {};
    if (stats.saved_target_valid) {
        format_ble_addr(stats.saved_addr, saved, sizeof(saved));
    }

    std::snprintf(out,
                  out_len,
                  "{\"ok\":true,\"profile\":\"ns2pro\",\"platform\":\"esp32s3\","
                  "\"state\":\"%s\",\"ready\":%s,\"connected\":%s,"
                  "\"scanning\":%s,\"candidates\":%lu,\"candidate_count\":%lu,"
                  "\"auto_connect\":%s,\"pair_mode\":%s,"
                  "\"saved_target\":%s,\"saved_addr\":\"%s\",\"saved_addr_type\":%u,"
                  "\"local_addr\":\"%s\",\"own_addr_type\":%u,"
                  "\"last_addr\":\"%s\",\"last_name\":\"%s\",\"rssi\":%d,"
                  "\"notify_count\":%lu,\"notify_hz\":%lu,\"last_notify_age_ms\":%lu,"
                  "\"notify_last_gap_us\":%lu,\"notify_max_gap_us\":%lu,"
                  "\"ble_conn_interval_units\":%u,\"ble_conn_interval_us\":%lu,"
                  "\"ble_conn_latency\":%u,\"ble_conn_supervision\":%u,"
                  "\"ble_conn_update_requests\":%lu,\"ble_conn_update_start_rc\":%d,"
                  "\"ble_conn_update_status\":%d,"
                  "\"tx_phy\":%u,\"rx_phy\":%u,\"phy_update_rc\":%d,"
                  "\"phy_update_status\":%d,\"data_len_update_rc\":%d,"
                  "\"data_len_tx_octets\":%u,\"data_len_rx_octets\":%u,"
                  "\"data_len_tx_time\":%u,\"data_len_rx_time\":%u,"
                  "\"connect_attempts\":%lu,\"disconnect_count\":%lu,"
                  "\"scan_starts\":%lu,\"scan_errors\":%lu,"
                  "\"adv_seen\":%lu,\"adv_logged\":%lu,\"last_error\":%d}",
                  stats.gatt_ready ? (input_last_age_ms() < 500 ? "connected_live" : "connected_no_notify") :
                      (stats.connected ? "connecting" : (stats.scanning ? "scanning" : (stats.started ? "idle" : "booting"))),
                  stats.gatt_ready ? "true" : "false",
                  stats.connected ? "true" : "false",
                  stats.scanning ? "true" : "false",
                  static_cast<unsigned long>(stats.candidates),
                  static_cast<unsigned long>(stats.candidates),
                  stats.auto_connect ? "true" : "false",
                  stats.pair_mode ? "true" : "false",
                  stats.saved_target_valid ? "true" : "false",
                  saved,
                  static_cast<unsigned>(stats.saved_addr_type),
                  local,
                  static_cast<unsigned>(stats.own_addr_type),
                  stats.last_addr,
                  stats.last_name,
                  static_cast<int>(stats.last_rssi),
                  static_cast<unsigned long>(stats.notify_count),
                  static_cast<unsigned long>(stats.notify_hz),
                  static_cast<unsigned long>(input_last_age_ms() == 0xffffffffu ? 0 : input_last_age_ms()),
                  static_cast<unsigned long>(stats.notify_last_gap_us),
                  static_cast<unsigned long>(stats.notify_max_gap_us),
                  static_cast<unsigned>(stats.conn_interval_units),
                  static_cast<unsigned long>(stats.conn_interval_us),
                  static_cast<unsigned>(stats.conn_latency),
                  static_cast<unsigned>(stats.conn_supervision_timeout),
                  static_cast<unsigned long>(stats.conn_update_requests),
                  stats.conn_update_start_rc,
                  stats.conn_update_status,
                  static_cast<unsigned>(stats.tx_phy),
                  static_cast<unsigned>(stats.rx_phy),
                  stats.phy_update_rc,
                  stats.phy_update_status,
                  stats.data_len_update_rc,
                  static_cast<unsigned>(stats.data_len_tx_octets),
                  static_cast<unsigned>(stats.data_len_rx_octets),
                  static_cast<unsigned>(stats.data_len_tx_time),
                  static_cast<unsigned>(stats.data_len_rx_time),
                  static_cast<unsigned long>(stats.connect_attempts),
                  static_cast<unsigned long>(stats.disconnect_count),
                  static_cast<unsigned long>(stats.scan_starts),
                  static_cast<unsigned long>(stats.scan_errors),
                  static_cast<unsigned long>(stats.adv_seen),
                  static_cast<unsigned long>(stats.adv_logged),
                  stats.last_error);
}

void format_motion_status_json(char *out, size_t out_len) {
    InputSnapshot input;
    const bool valid = input_get_snapshot(&input);
    char motion_json[384];
    input_format_motion_json(valid ? &input : nullptr, motion_json, sizeof(motion_json));
    char raw_head[33] = {};
    if (valid && input.raw_valid) {
        const size_t raw_head_len = std::min<size_t>(16, input.raw_len);
        for (size_t i = 0; i < raw_head_len; ++i) {
            std::snprintf(raw_head + i * 2, sizeof(raw_head) - i * 2, "%02x", input.raw[i]);
        }
    }
    std::snprintf(out,
                  out_len,
                  "{\"ok\":true,\"profile\":\"ns2pro\",\"platform\":\"esp32s3\","
                  "\"input_valid\":%s,\"kind\":\"%s\",\"len\":%u,"
                  "\"updates\":%lu,\"buttons\":%lu,\"lx\":%u,\"ly\":%u,"
                  "\"rx\":%u,\"ry\":%u,\"battery_valid\":%s,"
                  "\"battery_percent\":%u,\"battery_raw\":%u,"
                  "\"battery_offset\":%u,\"battery_charging\":%s,"
                  "\"raw_head\":\"%s\",%s}",
                  valid ? "true" : "false",
                  valid ? input_kind_name(input.kind) : "NEUTRAL",
                  valid ? static_cast<unsigned>(input.len) : 0u,
                  valid ? static_cast<unsigned long>(input.updates) : static_cast<unsigned long>(s_usb.reports_sent),
                  valid ? static_cast<unsigned long>(input.buttons) : 0ul,
                  valid ? static_cast<unsigned>(input.lx) : 2048u,
                  valid ? static_cast<unsigned>(input.ly) : 2048u,
                  valid ? static_cast<unsigned>(input.rx) : 2048u,
                  valid ? static_cast<unsigned>(input.ry) : 2048u,
                  valid && input.battery_valid ? "true" : "false",
                  valid ? static_cast<unsigned>(input.battery_percent) : 0u,
                  valid ? static_cast<unsigned>(input.battery_raw) : 0u,
                  valid ? static_cast<unsigned>(input.battery_offset) : 0xffu,
                  valid && input.battery_charging ? "true" : "false",
                  raw_head,
                  motion_json);
}

void format_rumble_config_json(char *out, size_t out_len) {
    std::snprintf(out,
                  out_len,
                  "{\"ok\":true,\"profile\":\"ns2pro\",\"platform\":\"esp32s3\","
                  "\"rumble_enabled\":%s,\"rumble_active\":%s,"
                  "\"output_mode\":\"%s\","
                  "\"scale_percent\":%u,\"hold_ms\":%u,\"tick_ms\":%u,"
                  "\"stop_packets\":%u,\"updates\":%lu,\"writes\":%lu,"
                  "\"stops\":%lu,\"errors\":%lu,\"switch_reports\":%lu,"
                  "\"ds5_reports\":%lu,\"xinput_reports\":%lu,"
                  "\"dual_motor_reports\":%lu,\"last_source\":\"%s\","
                  "\"last_left_heavy\":%u,\"last_right_light\":%u,"
                  "\"implemented\":true}",
                  s_usb.rumble_enabled ? "true" : "false",
                  s_usb.rumble_active ? "true" : "false",
                  config_output_mode_name(s_usb.output_mode),
                  static_cast<unsigned>(s_usb.rumble_scale_percent),
                  static_cast<unsigned>(s_usb.rumble_hold_ms),
                  static_cast<unsigned>(s_usb.rumble_tick_ms),
                  static_cast<unsigned>(s_usb.rumble_stop_packets),
                  static_cast<unsigned long>(s_usb.rumble_updates),
                  static_cast<unsigned long>(s_usb.rumble_writes),
                  static_cast<unsigned long>(s_usb.rumble_stops),
                  static_cast<unsigned long>(s_usb.rumble_errors),
                  static_cast<unsigned long>(s_usb.rumble_switch_reports),
                  static_cast<unsigned long>(s_usb.rumble_ds5_reports),
                  static_cast<unsigned long>(s_usb.rumble_xinput_reports),
                  static_cast<unsigned long>(s_usb.rumble_dual_motor_reports),
                  s_usb.rumble_last_source,
                  static_cast<unsigned>(s_usb.rumble_last_left_heavy),
                  static_cast<unsigned>(s_usb.rumble_last_right_light));
}

void handle_usb_command(const char *command) {
    static char json[1280];
    if (command_is(command, "usb status")) {
        format_usb_status_json(json, sizeof(json));
        queue_feature_json(json);
        return;
    }
    if (command_is(command, "usb config") || command_is(command, "report config")) {
        std::snprintf(json,
                      sizeof(json),
                      "{\"ok\":true,\"profile\":\"ns2pro\",\"platform\":\"esp32s3\","
                      "\"report_rate_hz\":%u,\"report_interval_us\":%lu,"
                      "\"reports_sent\":%lu,\"reports_failed\":%lu}",
                      static_cast<unsigned>(s_usb.report_rate_hz),
                      static_cast<unsigned long>(s_usb.report_interval_us),
                      static_cast<unsigned long>(s_usb.reports_sent),
                      static_cast<unsigned long>(s_usb.reports_failed));
        queue_feature_json(json);
        return;
    }
    if (command_is(command, "usb raw on")) {
        s_usb.raw_passthrough = true;
        format_settings_json(json, sizeof(json));
        queue_feature_json(json);
        return;
    }
    if (command_is(command, "usb raw off")) {
        s_usb.raw_passthrough = false;
        format_settings_json(json, sizeof(json));
        queue_feature_json(json);
        return;
    }
    if (command_is(command, "usb debug a on")) {
        s_usb.debug_force_a = true;
        queue_feature_json("{\"ok\":true,\"profile\":\"ns2pro\",\"platform\":\"esp32s3\",\"debug_force_a\":true}");
        return;
    }
    if (command_is(command, "usb debug a off")) {
        s_usb.debug_force_a = false;
        queue_feature_json("{\"ok\":true,\"profile\":\"ns2pro\",\"platform\":\"esp32s3\",\"debug_force_a\":false}");
        return;
    }
    if (command_is(command, "usb debug live on")) {
        s_usb.debug_live_log = true;
        s_usb.debug_live_logged = 0;
        queue_feature_json("{\"ok\":true,\"profile\":\"ns2pro\",\"platform\":\"esp32s3\",\"debug_live\":true}");
        return;
    }
    if (command_is(command, "usb debug live off")) {
        s_usb.debug_live_log = false;
        queue_feature_json("{\"ok\":true,\"profile\":\"ns2pro\",\"platform\":\"esp32s3\",\"debug_live\":false}");
        return;
    }
    if (command_has_prefix(command, "usb rate") || command_has_prefix(command, "report rate")) {
        const char *cursor = command_has_prefix(command, "usb rate") ?
            command + std::strlen("usb rate") :
            command + std::strlen("report rate");
        uint32_t rate = 0;
        if (!parse_next_uint(&cursor, &rate)) {
            queue_feature_error("usage_usb_rate_hz");
            return;
        }
        set_report_rate(rate);
        handle_usb_command("usb config");
        return;
    }
    if (command_has_prefix(command, "usb mode") || command_has_prefix(command, "output mode")) {
        const char *cursor = command_has_prefix(command, "usb mode") ?
            command + std::strlen("usb mode") :
            command + std::strlen("output mode");
        OutputMode mode = OutputMode::Nintendo;
        if (!parse_output_mode(cursor, &mode)) {
            queue_feature_error("usage_usb_mode_nintendo_xinput_dualsense");
            return;
        }
        s_usb.pending_output_mode = mode;
        s_usb.pending_output_mode_valid = true;
        config_set_usb(s_usb.raw_passthrough,
                       s_usb.web_parse_reports,
                       s_usb.rumble_enabled,
                       s_usb.rumble_scale_percent,
                       s_usb.rumble_hold_ms,
                       s_usb.rumble_tick_ms,
                       s_usb.rumble_stop_packets,
                       s_usb.report_rate_hz,
                       saved_output_mode());
        config_save();
        ESP_LOGW(TAG, "USB mode saved for next boot current=%s pending=%s restart_required=%u",
                 config_output_mode_name(s_usb.output_mode),
                 config_output_mode_name(saved_output_mode()),
                 output_mode_restart_required() ? 1u : 0u);
        format_settings_json(json, sizeof(json), true);
        queue_feature_json(json);
        return;
    }
    if (command_is(command, "usb reboot") || command_is(command, "usb restart")) {
        queue_feature_json("{\"ok\":true,\"profile\":\"ns2pro\",\"platform\":\"esp32s3\",\"rebooting\":true}");
        ESP_LOGW(TAG, "USB mode restart requested");
        esp_restart();
        return;
    }
    queue_feature_error("unknown_usb_command");
}

void handle_settings_command(const char *command) {
    static char json[512];
    if (command_is(command, "settings") ||
        command_is(command, "settings status") ||
        command_is(command, "config") ||
        command_is(command, "config status")) {
        format_settings_json(json, sizeof(json));
        queue_feature_json(json);
        return;
    }
    if (command_is(command, "settings save") || command_is(command, "config save")) {
        config_set_usb(s_usb.raw_passthrough,
                       s_usb.web_parse_reports,
                       s_usb.rumble_enabled,
                       s_usb.rumble_scale_percent,
                       s_usb.rumble_hold_ms,
                       s_usb.rumble_tick_ms,
                       s_usb.rumble_stop_packets,
                       s_usb.report_rate_hz,
                       saved_output_mode());
        config_save();
        ESP_LOGW(TAG, "settings saved report_rate=%u raw=%u web_parse=%u rumble=%u current=%s pending=%s",
                 static_cast<unsigned>(s_usb.report_rate_hz),
                 s_usb.raw_passthrough ? 1u : 0u,
                 s_usb.web_parse_reports ? 1u : 0u,
                 s_usb.rumble_enabled ? 1u : 0u,
                 config_output_mode_name(s_usb.output_mode),
                 config_output_mode_name(saved_output_mode()));
        format_settings_json(json, sizeof(json), true);
        queue_feature_json(json);
        return;
    }
    if (command_is(command, "web parse on") || command_is(command, "webui parse on")) {
        s_usb.web_parse_reports = true;
        format_settings_json(json, sizeof(json));
        queue_feature_json(json);
        return;
    }
    if (command_is(command, "web parse off") || command_is(command, "webui parse off")) {
        s_usb.web_parse_reports = false;
        format_settings_json(json, sizeof(json));
        queue_feature_json(json);
        return;
    }
    if (command_has_prefix(command, "output mode") ||
        command_has_prefix(command, "usb mode")) {
        handle_usb_command(command);
        return;
    }
    queue_feature_error("unknown_settings_command");
}

void handle_rumble_command(const char *command) {
    static char json[768];
    if (command_is(command, "rumble on")) {
        s_usb.rumble_enabled = true;
    } else if (command_is(command, "rumble off") || command_is(command, "rumble stop")) {
        s_usb.rumble_enabled = false;
        stop_rumble();
    } else if (command_is(command, "rumble hdtest") ||
               command_is(command, "rumble test") ||
               command_is(command, "rumble test both")) {
        inject_mode_rumble_test(true, true, 240, 520);
    } else if (command_is(command, "rumble test left")) {
        inject_mode_rumble_test(true, false, 240, 520);
    } else if (command_is(command, "rumble test right")) {
        inject_mode_rumble_test(false, true, 240, 520);
    } else if (command_is(command, "rumble test click")) {
        inject_mode_rumble_test(true, true, 70, 760);
    } else if (command_has_prefix(command, "rumble hold")) {
        const char *cursor = command + std::strlen("rumble hold");
        uint32_t hold = 0;
        if (!parse_next_uint(&cursor, &hold)) {
            hold = s_usb.rumble_hold_ms;
        }
        inject_mode_rumble_test(true, true, static_cast<uint16_t>(clamp_int(static_cast<int>(hold), 20, 2000)), 480);
    } else if (command_has_prefix(command, "rumble motors")) {
        const char *cursor = command + std::strlen("rumble motors");
        uint32_t left_heavy = 0;
        uint32_t right_light = 0;
        uint32_t hold = kHostRumbleHoldMs;
        if (!parse_next_uint(&cursor, &left_heavy) ||
            !parse_next_uint(&cursor, &right_light)) {
            queue_feature_error("usage_rumble_motors_left_heavy_right_light_hold_ms");
            return;
        }
        parse_next_uint(&cursor, &hold);
        inject_mode_rumble(
            static_cast<uint8_t>(clamp_int(static_cast<int>(left_heavy), 0, 255)),
            static_cast<uint8_t>(clamp_int(static_cast<int>(right_light), 0, 255)),
            static_cast<uint16_t>(clamp_int(static_cast<int>(hold), 20, 2000)));
    } else if (command_has_prefix(command, "rumble xinput")) {
        const char *cursor = command + std::strlen("rumble xinput");
        uint32_t left_heavy = 0;
        uint32_t right_light = 0;
        if (!parse_next_uint(&cursor, &left_heavy) ||
            !parse_next_uint(&cursor, &right_light)) {
            queue_feature_error("usage_rumble_xinput_left_heavy_right_light");
            return;
        }
        const uint8_t packet[5] = {
            0x00,
            0x08,
            0x00,
            static_cast<uint8_t>(clamp_int(static_cast<int>(left_heavy), 0, 255)),
            static_cast<uint8_t>(clamp_int(static_cast<int>(right_light), 0, 255)),
        };
        bridge_xinput_output_to_ble(packet, sizeof(packet));
    } else if (command_has_prefix(command, "rumble tune")) {
        const char *cursor = command + std::strlen("rumble tune");
        uint32_t scale = 0;
        uint32_t hold = 0;
        uint32_t tick = 0;
        uint32_t stops = 0;
        if (!parse_next_uint(&cursor, &scale) ||
            !parse_next_uint(&cursor, &hold) ||
            !parse_next_uint(&cursor, &tick) ||
            !parse_next_uint(&cursor, &stops)) {
            queue_feature_error("usage_rumble_tune_scale_hold_tick_stops");
            return;
        }
        s_usb.rumble_scale_percent = static_cast<uint16_t>(clamp_int(static_cast<int>(scale), 0, 100));
        s_usb.rumble_hold_ms = static_cast<uint16_t>(clamp_int(static_cast<int>(hold), 20, 2000));
        s_usb.rumble_tick_ms = static_cast<uint16_t>(clamp_int(static_cast<int>(tick), 4, 100));
        s_usb.rumble_stop_packets = static_cast<uint8_t>(clamp_int(static_cast<int>(stops), 1, 10));
    } else if (!command_is(command, "rumble config") &&
               !command_has_prefix(command, "rumble tune") &&
               !command_has_prefix(command, "rumble hold") &&
               !command_has_prefix(command, "rumble motors") &&
               !command_has_prefix(command, "rumble xinput") &&
               !command_has_prefix(command, "rumble test") &&
               !command_is(command, "rumble hdtest")) {
        queue_feature_error("unknown_rumble_command");
        return;
    }
    format_rumble_config_json(json, sizeof(json));
    queue_feature_json(json);
}

void handle_ns2_command(const char *command) {
    static char json[1280];
    if (command_is(command, "ns2 status") || command_is(command, "status")) {
        format_ble_status_json(json, sizeof(json));
        queue_feature_json(json);
        return;
    }
    if (command_is(command, "ns2 scan")) {
        ble_start_scan();
        format_ble_status_json(json, sizeof(json));
        queue_feature_json(json);
        return;
    }
    if (command_is(command, "ns2 pair")) {
        ble_pair();
        format_ble_status_json(json, sizeof(json));
        queue_feature_json(json);
        return;
    }
    if (command_is(command, "ns2 reconnect")) {
        ble_reconnect();
        format_ble_status_json(json, sizeof(json));
        queue_feature_json(json);
        return;
    }
    if (command_is(command, "ns2 disconnect")) {
        ble_disconnect();
        format_ble_status_json(json, sizeof(json));
        queue_feature_json(json);
        return;
    }
    if (command_is(command, "ns2 forget")) {
        ble_forget();
        format_ble_status_json(json, sizeof(json));
        queue_feature_json(json);
        return;
    }
    if (command_is(command, "ns2 auto on")) {
        ble_set_auto_connect(true);
        format_ble_status_json(json, sizeof(json));
        queue_feature_json(json);
        return;
    }
    if (command_is(command, "ns2 auto off")) {
        ble_set_auto_connect(false);
        format_ble_status_json(json, sizeof(json));
        queue_feature_json(json);
        return;
    }
    if (command_is(command, "ns2 candidates")) {
        format_ble_status_json(json, sizeof(json));
        queue_feature_json(json);
        return;
    }
    queue_feature_error("unknown_ns2_command");
}

void handle_feature_command(const char *raw_command) {
    const char *command = skip_spaces(raw_command);
    if (command[0] == 0) {
        queue_feature_error("empty_command");
        return;
    }

    if (command_is(command, "status") || command_has_prefix(command, "ns2")) {
        handle_ns2_command(command);
        return;
    }
    if (command_is(command, "usb status") ||
        command_has_prefix(command, "usb") ||
        command_has_prefix(command, "report") ||
        command_has_prefix(command, "output mode")) {
        handle_usb_command(command);
        return;
    }
    if (command_is(command, "motion status") || command_is(command, "imu status")) {
        static char json[1280];
        format_motion_status_json(json, sizeof(json));
        queue_feature_json(json);
        return;
    }
    if (command_has_prefix(command, "settings") ||
        command_has_prefix(command, "config") ||
        command_has_prefix(command, "web parse") ||
        command_has_prefix(command, "webui parse")) {
        handle_settings_command(command);
        return;
    }
    if (command_has_prefix(command, "rumble")) {
        handle_rumble_command(command);
        return;
    }
    if (command_is(command, "bootrom") || command_is(command, "bootsel")) {
        queue_feature_json("{\"ok\":false,\"profile\":\"ns2pro\",\"platform\":\"esp32s3\",\"error\":\"bootrom_not_implemented\"}");
        return;
    }
    queue_feature_error("unknown_command");
}

void receive_feature_command(const uint8_t *payload, uint16_t payload_size) {
    s_usb.feature_set_count++;
    hold_feature_quiet();
    if (payload == nullptr || payload_size == 0) {
        queue_feature_error("empty_feature_report");
        return;
    }
    if (payload[0] == kUsbReportIdFeature) {
        payload++;
        payload_size--;
    }
    if (payload_size < std::strlen(kFeatureSetMagic) ||
        std::memcmp(payload, kFeatureSetMagic, std::strlen(kFeatureSetMagic)) != 0) {
        queue_feature_error("bad_magic");
        return;
    }

    payload += std::strlen(kFeatureSetMagic);
    payload_size = static_cast<uint16_t>(payload_size - std::strlen(kFeatureSetMagic));
    while (payload_size > 0 && payload[payload_size - 1] == 0) {
        payload_size--;
    }

    const size_t copy_len = std::min<size_t>(payload_size, sizeof(s_usb.feature_last_command) - 1);
    std::memcpy(s_usb.feature_last_command, payload, copy_len);
    s_usb.feature_last_command[copy_len] = 0;
    ESP_LOGW(TAG, "feature command: %s", s_usb.feature_last_command);
    handle_feature_command(s_usb.feature_last_command);
    hold_feature_quiet();
}

uint16_t build_feature_report(uint8_t *buffer, uint16_t reqlen) {
    if (buffer == nullptr || reqlen == 0) {
        return 0;
    }
    s_usb.feature_get_count++;
    hold_feature_quiet();
    if (s_usb.feature_reply_len == 0) {
        char json[128];
        std::snprintf(json,
                      sizeof(json),
                      "{\"ok\":true,\"profile\":\"ns2pro\",\"platform\":\"esp32s3\",\"ready\":false}");
        queue_feature_json(json);
    }

    std::memset(buffer, 0, reqlen);
    if (reqlen < kFeaturePayloadOffset) {
        return reqlen;
    }

    std::memcpy(buffer, kFeatureReplyMagic, std::strlen(kFeatureReplyMagic));
    const uint16_t total = s_usb.feature_reply_len;
    uint16_t offset = s_usb.feature_reply_offset;
    if (offset > total) {
        offset = total;
    }
    const uint16_t remaining = static_cast<uint16_t>(total - offset);
    const uint16_t chunk_max = static_cast<uint16_t>(reqlen - kFeaturePayloadOffset);
    const uint16_t chunk_len = std::min<uint16_t>(remaining, chunk_max);

    buffer[6] = static_cast<uint8_t>(total & 0xff);
    buffer[7] = static_cast<uint8_t>((total >> 8) & 0xff);
    buffer[8] = static_cast<uint8_t>(offset & 0xff);
    buffer[9] = static_cast<uint8_t>((offset >> 8) & 0xff);
    buffer[10] = static_cast<uint8_t>(chunk_len);
    if (chunk_len > 0) {
        std::memcpy(buffer + kFeaturePayloadOffset, s_usb.feature_reply + offset, chunk_len);
        s_usb.feature_reply_offset = static_cast<uint16_t>(offset + chunk_len);
        if (s_usb.feature_reply_offset >= total) {
            s_usb.feature_reply_len = 0;
            s_usb.feature_reply_offset = 0;
        } else {
            hold_feature_quiet();
        }
    }
    return reqlen;
}

void usb_event_cb(tinyusb_event_t *event, void *arg) {
    (void)arg;
    if (event == nullptr) {
        return;
    }
    if (event->id == TINYUSB_EVENT_ATTACHED) {
        s_usb.mounted = true;
        s_usb.suspended = false;
        s_usb.hid_guard_active = false;
        s_usb.hid_guard_done = false;
        s_usb.hid_guard_release_after_tx = false;
        s_usb.hid_guard_started_us = 0;
    } else if (event->id == TINYUSB_EVENT_DETACHED) {
        s_usb.mounted = false;
        s_usb.suspended = false;
        s_usb.hid_guard_active = false;
        s_usb.hid_guard_release_after_tx = false;
        s_usb.vendor_pending_len = 0;
        s_usb.vendor_pending_offset = 0;
    }
}

} // namespace

esp_err_t usb_start() {
    if (s_usb.started) {
        return ESP_OK;
    }

    usb_uac1_link_driver();
    RuntimeConfig cfg_runtime;
    config_get(&cfg_runtime);
    s_usb.raw_passthrough = cfg_runtime.raw_passthrough;
    s_usb.web_parse_reports = cfg_runtime.web_parse_reports;
    s_usb.rumble_enabled = cfg_runtime.rumble_enabled;
    s_usb.output_mode = cfg_runtime.output_mode;
    s_usb.pending_output_mode = cfg_runtime.output_mode;
    s_usb.pending_output_mode_valid = true;
    s_usb.rumble_scale_percent = cfg_runtime.rumble_scale_percent;
    s_usb.rumble_hold_ms = cfg_runtime.rumble_hold_ms;
    s_usb.rumble_tick_ms = cfg_runtime.rumble_tick_ms;
    s_usb.rumble_stop_packets = cfg_runtime.rumble_stop_packets;
    s_usb.ds5_regular_right = 0;
    s_usb.ds5_regular_left = 0;
    reset_ds5_haptics_state();
    if (s_ds5_audio_queue == nullptr) {
        s_ds5_audio_queue = xQueueCreateStatic(
            kDs5AudioQueueDepth,
            sizeof(Ds5AudioPacket),
            s_ds5_audio_queue_buffer,
            &s_ds5_audio_queue_storage);
        ESP_RETURN_ON_FALSE(s_ds5_audio_queue != nullptr, ESP_ERR_NO_MEM, TAG, "DS5 audio queue allocation failed");
    } else {
        xQueueReset(s_ds5_audio_queue);
    }
    set_report_rate(cfg_runtime.report_rate_hz == 0 ? kDefaultReportRateHz : cfg_runtime.report_rate_hz);
    tinyusb_config_t cfg = TINYUSB_CONFIG_FULL_SPEED(usb_event_cb, nullptr);
    cfg.descriptor.device = current_device_descriptor();
    cfg.descriptor.string = current_string_descriptor();
    cfg.descriptor.string_count = current_string_descriptor_count();
#if (TUD_OPT_HIGH_SPEED)
    cfg.descriptor.full_speed_config = current_configuration_descriptor();
    cfg.descriptor.high_speed_config = current_configuration_descriptor();
    cfg.descriptor.qualifier = nullptr;
#else
    cfg.descriptor.full_speed_config = current_configuration_descriptor();
#endif

    ESP_RETURN_ON_ERROR(tinyusb_driver_install(&cfg), TAG, "tinyusb_driver_install failed");
    s_usb.started = true;
    ESP_LOGI(TAG, "TinyUSB initialized mode=%s VID=%04x PID=%04x",
             config_output_mode_name(s_usb.output_mode),
             current_device_descriptor()->idVendor,
             current_device_descriptor()->idProduct);
    return ESP_OK;
}

void usb_note_uac1_interface(uint8_t interface_number, uint8_t alt_setting) {
    s_usb.audio_set_interface_count++;
    s_usb.audio_last_interface = interface_number;
    s_usb.audio_last_alt = alt_setting;
}

void usb_submit_dualsense_audio_packet(const uint8_t *data, uint16_t len) {
    if (data == nullptr || len == 0 || len > kDs5AudioPacketBytes ||
        s_usb.output_mode != OutputMode::DualSense || s_ds5_audio_queue == nullptr) {
        return;
    }

    Ds5AudioPacket packet{};
    packet.len = len;
    std::memcpy(packet.data, data, len);
    if (xQueueSend(s_ds5_audio_queue, &packet, 0) == pdTRUE) {
        return;
    }

    Ds5AudioPacket dropped{};
    xQueueReceive(s_ds5_audio_queue, &dropped, 0);
    xQueueSend(s_ds5_audio_queue, &packet, 0);
}

const int16_t kDs5GoertzelCoeffQ14[] = {
    0, 32610, 32138, 31357, 30274, 28899, 27246,
    25330, 23170, 20788, 18205, 15447, 12540, 9512,
};

uint32_t isqrt_u64(uint64_t value) {
    uint64_t bit = 1ULL << 62;
    uint32_t result = 0;
    while (bit > value) {
        bit >>= 2;
    }
    while (bit != 0) {
        if (value >= static_cast<uint64_t>(result) + bit) {
            value -= static_cast<uint64_t>(result) + bit;
            result = static_cast<uint32_t>((static_cast<uint64_t>(result) >> 1) + bit);
        } else {
            result >>= 1;
        }
        bit >>= 2;
    }
    return result;
}

uint64_t ds5_spectral_bin_power(const int16_t *samples, uint8_t bin) {
    constexpr size_t kCoeffCount = sizeof(kDs5GoertzelCoeffQ14) / sizeof(kDs5GoertzelCoeffQ14[0]);
    if (samples == nullptr || bin == 0 || bin >= kCoeffCount) {
        return 0;
    }
    int64_t s1 = 0;
    int64_t s2 = 0;
    const int32_t coeff = kDs5GoertzelCoeffQ14[bin];
    for (uint16_t i = 0; i < kDs5HapticsBufferLen; ++i) {
        const int64_t sample = samples[i] >> 4;
        const int64_t s0 = sample + ((coeff * s1) >> 14) - s2;
        s2 = s1;
        s1 = s0;
    }
    const int64_t power = s1 * s1 + s2 * s2 - ((coeff * s1 * s2) >> 14);
    return power > 0 ? static_cast<uint64_t>(power) : 0;
}

void ds5_analyze_band(const int16_t *samples,
                      uint8_t min_bin,
                      uint8_t max_bin,
                      uint16_t fallback_hz,
                      uint16_t *frequency,
                      uint16_t *rms) {
    uint64_t band_power = 0;
    uint64_t peak_power = 0;
    uint8_t peak_bin = 0;
    for (uint8_t bin = min_bin; bin <= max_bin; ++bin) {
        const uint64_t power = ds5_spectral_bin_power(samples, bin);
        band_power += power;
        if (power > peak_power) {
            peak_power = power;
            peak_bin = bin;
        }
    }
    if (band_power == 0 || peak_bin == 0) {
        *frequency = fallback_hz;
        *rms = 0;
        return;
    }
    uint64_t rms_scaled = isqrt_u64(band_power * 2u) / kDs5HapticsBufferLen;
    rms_scaled <<= 4;
    *rms = static_cast<uint16_t>(std::min<uint64_t>(rms_scaled, UINT16_MAX));
    *frequency = static_cast<uint16_t>(
        (static_cast<uint32_t>(peak_bin) * kDs5HapticsSampleRateHz + kDs5HapticsBufferLen / 2u) /
        kDs5HapticsBufferLen);
}

void ds5_analyze_side(const int16_t *input,
                      uint16_t *low_freq,
                      uint16_t *high_freq,
                      uint16_t *low_rms,
                      uint16_t *high_rms) {
    int16_t centered[kDs5HapticsBufferLen];
    int64_t sum = 0;
    for (uint16_t i = 0; i < kDs5HapticsBufferLen; ++i) {
        sum += input[i];
    }
    const int32_t mean = static_cast<int32_t>(sum / kDs5HapticsBufferLen);
    for (uint16_t i = 0; i < kDs5HapticsBufferLen; ++i) {
        centered[i] = static_cast<int16_t>(static_cast<int32_t>(input[i]) - mean);
    }

    uint16_t detected_low = *low_freq;
    uint16_t detected_high = *high_freq;
    ds5_analyze_band(centered, kDs5HapticsLowBinMin, kDs5HapticsLowBinMax, *low_freq, &detected_low, low_rms);
    ds5_analyze_band(centered, kDs5HapticsHighBinMin, kDs5HapticsHighBinMax, *high_freq, &detected_high, high_rms);
    *low_freq = static_cast<uint16_t>((static_cast<uint32_t>(*low_freq) + detected_low + 1u) / 2u);
    *high_freq = static_cast<uint16_t>((static_cast<uint32_t>(*high_freq) + detected_high + 1u) / 2u);
}

uint8_t ds5_rms_to_status_strength(uint16_t rms) {
    const uint16_t amplitude = map_ds5_spectral_amplitude(rms);
    return static_cast<uint8_t>((static_cast<uint32_t>(amplitude) * 255u + 511u) / 1023u);
}

void process_ds5_haptics_window() {
    ds5_analyze_side(
        s_usb.ds5_haptics_left_samples,
        &s_usb.ds5_haptic_left_low_freq,
        &s_usb.ds5_haptic_left_high_freq,
        &s_usb.ds5_haptic_left_low_rms,
        &s_usb.ds5_haptic_left_high_rms);
    ds5_analyze_side(
        s_usb.ds5_haptics_right_samples,
        &s_usb.ds5_haptic_right_low_freq,
        &s_usb.ds5_haptic_right_high_freq,
        &s_usb.ds5_haptic_right_low_rms,
        &s_usb.ds5_haptic_right_high_rms);

    s_usb.ds5_haptic_left_low = ds5_rms_to_status_strength(s_usb.ds5_haptic_left_low_rms);
    s_usb.ds5_haptic_left_high = ds5_rms_to_status_strength(s_usb.ds5_haptic_left_high_rms);
    s_usb.ds5_haptic_right_low = ds5_rms_to_status_strength(s_usb.ds5_haptic_right_low_rms);
    s_usb.ds5_haptic_right_high = ds5_rms_to_status_strength(s_usb.ds5_haptic_right_high_rms);
    update_ds5_rumble_mix("ds5_audio", 80);
}

void push_ds5_haptics_sample(int16_t left, int16_t right) {
    const uint8_t position = s_usb.ds5_haptics_sample_pos;
    s_usb.ds5_haptics_left_samples[position] = left;
    s_usb.ds5_haptics_right_samples[position] = right;
    s_usb.ds5_haptics_sample_pos++;
    if (s_usb.ds5_haptics_sample_pos < kDs5HapticsBufferLen) {
        return;
    }

    process_ds5_haptics_window();
    std::memmove(
        s_usb.ds5_haptics_left_samples,
        s_usb.ds5_haptics_left_samples + kDs5HapticsWindowHop,
        (kDs5HapticsBufferLen - kDs5HapticsWindowHop) * sizeof(int16_t));
    std::memmove(
        s_usb.ds5_haptics_right_samples,
        s_usb.ds5_haptics_right_samples + kDs5HapticsWindowHop,
        (kDs5HapticsBufferLen - kDs5HapticsWindowHop) * sizeof(int16_t));
    s_usb.ds5_haptics_sample_pos = kDs5HapticsBufferLen - kDs5HapticsWindowHop;
}

void audio_task() {
    if (s_usb.output_mode != OutputMode::DualSense || !s_usb.started || !tud_mounted() ||
        s_ds5_audio_queue == nullptr) {
        return;
    }

    Ds5AudioPacket packet{};
    for (uint8_t queued = 0; queued < 8 && xQueueReceive(s_ds5_audio_queue, &packet, 0) == pdTRUE; queued++) {
        s_usb.audio_out_packets++;
        s_usb.audio_out_bytes += packet.len;
        s_usb.audio_last_read = packet.len;

        const int16_t *samples = reinterpret_cast<const int16_t *>(packet.data);
        const uint32_t sample_count = packet.len / sizeof(samples[0]);
        const uint32_t frames = sample_count / 4;
        for (uint32_t i = 0; i < frames; i++) {
            s_usb.ds5_audio_left_sum += samples[i * 4 + 2];
            s_usb.ds5_audio_right_sum += samples[i * 4 + 3];
            s_usb.ds5_audio_decim_count++;
            if (s_usb.ds5_audio_decim_count < kDs5AudioDecimation) {
                continue;
            }

            const int32_t left_avg = s_usb.ds5_audio_left_sum / kDs5AudioDecimation;
            const int32_t right_avg = s_usb.ds5_audio_right_sum / kDs5AudioDecimation;
            push_ds5_haptics_sample(
                static_cast<int16_t>(clamp_int(static_cast<int>(left_avg), INT16_MIN, INT16_MAX)),
                static_cast<int16_t>(clamp_int(static_cast<int>(right_avg), INT16_MIN, INT16_MAX)));
            s_usb.ds5_audio_decim_count = 0;
            s_usb.ds5_audio_left_sum = 0;
            s_usb.ds5_audio_right_sum = 0;
        }

        if (s_usb.audio_logged < 16) {
            ESP_LOGW(TAG,
                     "ds5 audio bytes=%u frames=%lu haptic=%u/%u/%u/%u freq=%u/%u/%u/%u total=%lu",
                     static_cast<unsigned>(packet.len),
                     static_cast<unsigned long>(frames),
                     static_cast<unsigned>(s_usb.ds5_haptic_left_low),
                     static_cast<unsigned>(s_usb.ds5_haptic_left_high),
                     static_cast<unsigned>(s_usb.ds5_haptic_right_low),
                     static_cast<unsigned>(s_usb.ds5_haptic_right_high),
                     static_cast<unsigned>(s_usb.ds5_haptic_left_low_freq),
                     static_cast<unsigned>(s_usb.ds5_haptic_left_high_freq),
                     static_cast<unsigned>(s_usb.ds5_haptic_right_low_freq),
                     static_cast<unsigned>(s_usb.ds5_haptic_right_high_freq),
                     static_cast<unsigned long>(s_usb.audio_out_packets));
            s_usb.audio_logged++;
        }
    }
}

void usb_task() {
    rumble_task();
    audio_task();

    if (!s_usb.started || !tud_mounted()) {
        return;
    }
    if (hid_guard_active()) {
        return;
    }

    const uint64_t now = esp_timer_get_time();
    if (s_usb.feature_quiet_until_us != 0 && now < s_usb.feature_quiet_until_us) {
        return;
    }
    if (s_usb.next_report_us != 0 && now < s_usb.next_report_us) {
        return;
    }

    InputSnapshot input;
    const bool live = input_get_snapshot(&input) && input_last_age_ms() < 500;
    check_mode_combo(&input, live, now);
    const uint32_t input_updates = live ? input.updates : 0;
    const bool new_input = live && input_updates != s_usb.last_reported_input_updates;
    const bool keepalive_due = s_usb.report_last_sent_us == 0 ||
                               now - s_usb.report_last_sent_us >= kUsbKeepaliveIntervalUs;
    if (!new_input && !keepalive_due) {
        return;
    }

    s_usb.next_report_us = now + s_usb.report_interval_us;
    bool sent = false;
    if (s_usb.output_mode == OutputMode::XInput) {
        if (tud_vendor_n_mounted(kVendorInstance) && tud_vendor_n_write_available(kVendorInstance) >= kXInputReportLen) {
            uint8_t report[kXInputReportLen] = {};
            fill_xinput_report(live ? &input : nullptr, report);
            const uint32_t written = tud_vendor_n_write(kVendorInstance, report, sizeof(report));
            tud_vendor_n_write_flush(kVendorInstance);
            sent = written == sizeof(report);
        }
    } else if (tud_hid_ready()) {
        if (s_usb.output_mode == OutputMode::DualSense) {
            uint8_t report[kDualSenseInputPayloadLen] = {};
            fill_dualsense_report(live ? &input : nullptr, report);
            sent = tud_hid_report(kDualSenseInputReportId, report, sizeof(report));
        } else {
            uint8_t report[kInputReportPayloadLen] = {};
            fill_input_report(live ? &input : nullptr, report);
            sent = tud_hid_report(kUsbReportIdInput, report, sizeof(report));
        }
    }
    if (sent) {
        s_usb.last_reported_input_updates = input_updates;
        s_usb.reports_sent++;
        if (s_usb.report_last_sent_us != 0) {
            const uint32_t gap = static_cast<uint32_t>(now - s_usb.report_last_sent_us);
            s_usb.report_last_gap_us = gap;
            if (gap > s_usb.report_max_gap_us) {
                s_usb.report_max_gap_us = gap;
            }
        }
        s_usb.report_last_sent_us = now;
        if (s_usb.report_rate_sample_us == 0) {
            s_usb.report_rate_sample_us = now;
            s_usb.report_rate_sample_count = s_usb.reports_sent;
        } else {
            const uint64_t elapsed = now - s_usb.report_rate_sample_us;
            if (elapsed >= 1000000ULL) {
                const uint32_t delta = s_usb.reports_sent - s_usb.report_rate_sample_count;
                s_usb.report_submit_hz = static_cast<uint32_t>((static_cast<uint64_t>(delta) * 1000000ULL) / elapsed);
                s_usb.report_rate_sample_us = now;
                s_usb.report_rate_sample_count = s_usb.reports_sent;
                s_usb.report_max_gap_us = 0;
            }
        }
    } else {
        s_usb.reports_failed++;
    }
}

} // namespace ns2

extern "C" void tud_suspend_cb(bool remote_wakeup_en) {
    (void)remote_wakeup_en;
    ns2::s_usb.suspended = true;
}

extern "C" void tud_resume_cb(void) {
    ns2::s_usb.suspended = false;
}

extern "C" uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance) {
    if (ns2::s_usb.output_mode == ns2::OutputMode::DualSense &&
        instance == ns2::kGamepadHidInstance) {
        return ns2::kDualSenseReportDescriptor;
    }
    return ns2::kHidReportDescriptor;
}

extern "C" uint16_t tud_hid_get_report_cb(uint8_t instance,
                                           uint8_t report_id,
                                           hid_report_type_t report_type,
                                           uint8_t *buffer,
                                           uint16_t reqlen) {
    if (report_type == HID_REPORT_TYPE_FEATURE &&
        !(ns2::s_usb.output_mode == ns2::OutputMode::DualSense &&
          instance == ns2::kGamepadHidInstance) &&
        (report_id == 0 || report_id == ns2::kUsbReportIdFeature)) {
        return ns2::build_feature_report(buffer, reqlen);
    }
    if (report_type == HID_REPORT_TYPE_INPUT &&
        ns2::s_usb.output_mode == ns2::OutputMode::DualSense &&
        instance == ns2::kGamepadHidInstance &&
        (report_id == 0 || report_id == ns2::kDualSenseInputReportId)) {
        uint8_t report[ns2::kDualSenseInputPayloadLen] = {};
        ns2::fill_dualsense_report(nullptr, report);
        const uint16_t copy_len = std::min<uint16_t>(reqlen, sizeof(report));
        std::memcpy(buffer, report, copy_len);
        return copy_len;
    }
    if (report_type == HID_REPORT_TYPE_INPUT &&
        ns2::s_usb.output_mode == ns2::OutputMode::Nintendo &&
        instance == ns2::kGamepadHidInstance &&
        (report_id == 0 || report_id == ns2::kUsbReportIdInput)) {
        uint8_t report[ns2::kInputReportPayloadLen] = {};
        ns2::fill_neutral_input(report);
        const uint16_t copy_len = std::min<uint16_t>(reqlen, sizeof(report));
        std::memcpy(buffer, report, copy_len);
        return copy_len;
    }
    return 0;
}

extern "C" void tud_hid_set_report_cb(uint8_t instance,
                                       uint8_t report_id,
                                       hid_report_type_t report_type,
                                       uint8_t const *buffer,
                                       uint16_t bufsize) {
    uint8_t effective_report_id = report_id;
    const uint8_t *payload = buffer;
    uint16_t payload_size = bufsize;
    if (effective_report_id == 0 && buffer != nullptr && bufsize > 0) {
        effective_report_id = buffer[0];
        payload = buffer + 1;
        payload_size = static_cast<uint16_t>(bufsize - 1);
    }

    ns2::s_usb.hid_last_report_id = effective_report_id;
    ns2::s_usb.hid_last_type = static_cast<uint8_t>(report_type);
    ns2::s_usb.hid_last_len = bufsize;

    if (report_type == HID_REPORT_TYPE_FEATURE &&
        !(ns2::s_usb.output_mode == ns2::OutputMode::DualSense &&
          instance == ns2::kGamepadHidInstance) &&
        effective_report_id == ns2::kUsbReportIdFeature) {
        ns2::receive_feature_command(payload, payload_size);
        return;
    }

    if (effective_report_id == ns2::kUsbReportIdOutput &&
        (ns2::s_usb.output_mode != ns2::OutputMode::DualSense ||
         instance == ns2::kGamepadHidInstance)) {
        ns2::s_usb.hid_out_count++;
        uint8_t full_report[64] = {};
        full_report[0] = effective_report_id;
        const uint16_t copy_len = std::min<uint16_t>(payload_size, sizeof(full_report) - 1);
        if (payload != nullptr && copy_len > 0) {
            std::memcpy(full_report + 1, payload, copy_len);
        }
        const uint16_t full_len = static_cast<uint16_t>(copy_len + 1);
        if (!ns2::bridge_hid_output_to_ble(full_report, full_len)) {
            ns2::bridge_ds5_output_to_ble(effective_report_id, payload, payload_size);
        }
        ESP_LOGD(ns2::TAG, "output report type=%u size=%u", report_type, payload_size);
    }
}

extern "C" void tud_hid_report_complete_cb(uint8_t instance, uint8_t const *report, uint16_t len) {
    if (instance != ns2::kGamepadHidInstance) {
        return;
    }
    const uint8_t expected_id = ns2::s_usb.output_mode == ns2::OutputMode::DualSense ?
        ns2::kDualSenseInputReportId :
        ns2::kUsbReportIdInput;
    if (report == nullptr || len == 0 || report[0] != expected_id) {
        return;
    }

    const uint64_t now = esp_timer_get_time();
    ns2::s_usb.reports_completed++;
    if (ns2::s_usb.report_last_complete_us != 0) {
        const uint32_t gap = static_cast<uint32_t>(now - ns2::s_usb.report_last_complete_us);
        ns2::s_usb.report_complete_last_gap_us = gap;
        if (gap > ns2::s_usb.report_complete_max_gap_us) {
            ns2::s_usb.report_complete_max_gap_us = gap;
        }
    }
    ns2::s_usb.report_last_complete_us = now;

    if (ns2::s_usb.report_complete_sample_us == 0) {
        ns2::s_usb.report_complete_sample_us = now;
        ns2::s_usb.report_complete_sample_count = ns2::s_usb.reports_completed;
        return;
    }
    const uint64_t elapsed = now - ns2::s_usb.report_complete_sample_us;
    if (elapsed >= 1000000ULL) {
        const uint32_t delta = ns2::s_usb.reports_completed - ns2::s_usb.report_complete_sample_count;
        ns2::s_usb.report_complete_hz =
            static_cast<uint32_t>((static_cast<uint64_t>(delta) * 1000000ULL) / elapsed);
        ns2::s_usb.report_complete_sample_us = now;
        ns2::s_usb.report_complete_sample_count = ns2::s_usb.reports_completed;
        ns2::s_usb.report_complete_max_gap_us = 0;
    }
}

extern "C" uint8_t const *tud_descriptor_bos_cb(void) {
    return ns2::s_usb.output_mode == ns2::OutputMode::Nintendo ? ns2::kBosDescriptor : nullptr;
}

extern "C" uint16_t const *tinyusb_extra_string_descriptor_cb(uint8_t index, uint16_t langid) {
    (void)langid;
    if (index == ns2::kMsOs10StringIndex) {
        ESP_LOGW(ns2::TAG, "MS OS 1.0 string descriptor requested");
        return reinterpret_cast<uint16_t const *>(ns2::kMsOs10StringDescriptor);
    }
    return nullptr;
}

extern "C" bool tud_vendor_control_xfer_cb(uint8_t rhport,
                                            uint8_t stage,
                                            tusb_control_request_t const *request) {
    if (stage != CONTROL_STAGE_SETUP) {
        return true;
    }
    if (request == nullptr) {
        return false;
    }
    if (request->bmRequestType_bit.type != TUSB_REQ_TYPE_VENDOR ||
        request->bRequest != ns2::kMsVendorCode) {
        return false;
    }

    if (request->wIndex == 0x0004) {
        ESP_LOGW(ns2::TAG, "MS OS 1.0 compat ID requested");
        return tud_control_xfer(rhport,
                                request,
                                const_cast<uint8_t *>(ns2::kMsOs10CompatIdDescriptor),
                                sizeof(ns2::kMsOs10CompatIdDescriptor));
    }
    if (request->wIndex == 0x0005) {
        ESP_LOGW(ns2::TAG, "MS OS 1.0 property requested");
        return tud_control_xfer(rhport,
                                request,
                                const_cast<uint8_t *>(ns2::kMsOs10PropertyDescriptor),
                                sizeof(ns2::kMsOs10PropertyDescriptor));
    }
    if (request->wIndex == 0x0007) {
        ESP_LOGW(ns2::TAG, "MS OS 2.0 descriptor requested");
        return tud_control_xfer(rhport,
                                request,
                                const_cast<uint8_t *>(ns2::kMsOs20Descriptor),
                                ns2::kMsOs20DescriptorLen);
    }
    return false;
}

extern "C" void tud_vendor_rx_cb(uint8_t itf, uint8_t const *buffer, uint16_t bufsize) {
    if (itf != ns2::kVendorInstance || buffer == nullptr || bufsize == 0) {
        return;
    }

    uint8_t cmd[ns2::kBulkReplyMax];
    const uint16_t cmd_len = std::min<uint16_t>(bufsize, sizeof(cmd));
    std::memcpy(cmd, buffer, cmd_len);
    tud_vendor_n_read_flush(itf);

    ns2::s_usb.vendor_out_count++;
    ns2::s_usb.vendor_last_rx_len = cmd_len;
    ns2::s_usb.vendor_last_cmd = cmd[0];
    ns2::s_usb.vendor_last_arg = cmd_len > 3 ? cmd[3] : 0;
    ns2::s_usb.vendor_last_address = ns2::command_address(cmd, cmd_len);

    if (ns2::bridge_xinput_output_to_ble(cmd, cmd_len)) {
        ns2::s_usb.vendor_last_tx_len = 0;
        return;
    }

    if (!ns2::bridge_hid_output_to_ble(cmd, cmd_len)) {
        // Non-rumble vendor traffic falls through to the manager reply path.
    }

    uint8_t reply[ns2::kBulkReplyMax];
    const size_t reply_len = ns2::build_vendor_reply(cmd, cmd_len, reply, sizeof(reply));
    ns2::s_usb.vendor_last_tx_len = static_cast<uint16_t>(reply_len);
    if (reply_len == 0) {
        return;
    }
    if (cmd[0] == 0x03 && ns2::s_usb.vendor_last_arg == 0x0d) {
        ns2::s_usb.hid_guard_release_after_tx = true;
    }
    ns2::s_usb.vendor_in_count++;
    ns2::queue_vendor_reply(itf, reply, reply_len);
    ESP_LOGW(ns2::TAG,
             "vendor bulk cmd=%02x arg=%02x rx=%u tx=%u addr=%08lx",
             ns2::s_usb.vendor_last_cmd,
             ns2::s_usb.vendor_last_arg,
             static_cast<unsigned>(cmd_len),
             static_cast<unsigned>(reply_len),
             static_cast<unsigned long>(ns2::s_usb.vendor_last_address));
}

extern "C" void tud_vendor_tx_cb(uint8_t itf, uint32_t sent_bytes) {
    ns2::s_usb.vendor_in_done_count++;
    ns2::s_usb.vendor_last_sent_bytes = sent_bytes;
    if (ns2::s_usb.hid_guard_release_after_tx) {
        ns2::release_hid_guard("start output ack sent");
    }
    if (itf == ns2::s_usb.vendor_pending_itf) {
        ns2::flush_vendor_pending();
    }
}
