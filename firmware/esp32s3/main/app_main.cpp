#include "esp_chip_info.h"
#include "esp_err.h"
#include "esp_log.h"

#include <algorithm>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ns2_ble.h"
#include "ns2_board.h"
#include "ns2_config.h"
#include "ns2_usb.h"

namespace {

const char *TAG = "ns2pro-esp32s3";

} // namespace

extern "C" void app_main(void) {
    esp_chip_info_t chip{};
    esp_chip_info(&chip);

    ESP_LOGI(TAG, "ns2pro-bridge ESP32-S3 scaffold booted");
    ESP_LOGI(TAG,
             "chip cores=%d features=0x%lx revision=%d",
             chip.cores,
             static_cast<unsigned long>(chip.features),
             chip.revision);
    ESP_ERROR_CHECK(ns2::config_init());
    ESP_ERROR_CHECK(ns2::board_controls_init());
    ESP_ERROR_CHECK(ns2::usb_start());

    const esp_err_t ble_rc = ns2::ble_start();
    if (ble_rc != ESP_OK) {
        ESP_LOGE(TAG, "BLE startup failed: %s; keeping USB control path alive", esp_err_to_name(ble_rc));
    }

    ESP_LOGW(TAG,
             "Experimental ESP32-S3 build: BLE connect/GATT/input/rumble paths are compiled in; "
             "real controller behavior still needs hardware validation");

    while (true) {
        ns2::ble_task();
        ns2::usb_task();
        ns2::board_controls_task();
        vTaskDelay(std::max<TickType_t>(1, pdMS_TO_TICKS(1)));
    }
}
