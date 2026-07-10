#pragma once

#include <cstdint>

#include "esp_err.h"

namespace ns2 {

esp_err_t usb_start();
void usb_task();
void usb_uac1_link_driver();
void usb_submit_dualsense_audio_packet(const uint8_t *data, uint16_t len);
void usb_note_uac1_interface(uint8_t interface_number, uint8_t alt_setting);

} // namespace ns2
