#include "ns2_board.h"

#include "ns2_ble.h"
#include "ns2_config.h"

#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "led_strip.h"
#include "sdkconfig.h"

namespace ns2 {
namespace {

const char *TAG = "ns2-board";

constexpr int64_t kButtonDebounceUs = 30LL * 1000LL;
constexpr int64_t kDoubleClickWindowUs = 450LL * 1000LL;
constexpr int64_t kLedUpdateUs = 100LL * 1000LL;
constexpr int64_t kDisconnectedBlinkUs = 500LL * 1000LL;
constexpr uint8_t kLedBrightness = 24;

struct Rgb {
    uint8_t r;
    uint8_t g;
    uint8_t b;
};

struct BoardState {
    bool initialized = false;
    bool button_level = true;
    bool button_stable_level = true;
    int64_t button_changed_us = 0;
    int64_t first_click_us = 0;
    uint8_t click_count = 0;
    int64_t last_led_update_us = 0;
    Rgb last_color = {0, 0, 0};
#if CONFIG_NS2_STATUS_LED_ENABLE
    led_strip_handle_t led_strip = nullptr;
#endif
};

BoardState s_board;

OutputMode next_mode(OutputMode mode) {
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

void switch_mode_from_button() {
    const OutputMode current = config_output_mode();
    const OutputMode next = next_mode(current);
    config_set_output_mode(next);
    config_save();
    ESP_LOGW(TAG, "BOOT double-click: switching USB mode %s -> %s",
             config_output_mode_name(current),
             config_output_mode_name(next));
    esp_restart();
}

void poll_boot_button(int64_t now) {
    const bool level = gpio_get_level(static_cast<gpio_num_t>(CONFIG_NS2_BOOT_BUTTON_GPIO)) != 0;
    if (level != s_board.button_level) {
        s_board.button_level = level;
        s_board.button_changed_us = now;
        return;
    }
    if (level == s_board.button_stable_level || now - s_board.button_changed_us < kButtonDebounceUs) {
        if (s_board.click_count == 1 && now - s_board.first_click_us > kDoubleClickWindowUs) {
            s_board.click_count = 0;
            s_board.first_click_us = 0;
        }
        return;
    }

    s_board.button_stable_level = level;
    if (!level) {
        return;
    }

    if (s_board.click_count == 0 || now - s_board.first_click_us > kDoubleClickWindowUs) {
        s_board.click_count = 1;
        s_board.first_click_us = now;
        return;
    }

    s_board.click_count = 0;
    s_board.first_click_us = 0;
    switch_mode_from_button();
}

Rgb mode_color(OutputMode mode) {
    switch (mode) {
    case OutputMode::Nintendo:
        return {kLedBrightness, 0, 0};
    case OutputMode::DualSense:
        return {0, 0, kLedBrightness};
    case OutputMode::XInput:
        return {0, kLedBrightness, 0};
    default:
        return {0, 0, 0};
    }
}

Rgb status_color(int64_t now) {
    BleStats stats{};
    ble_get_stats(&stats);
    const Rgb color = mode_color(config_output_mode());
    if (!stats.gatt_ready) {
        const bool on = ((now / kDisconnectedBlinkUs) & 1) == 0;
        return on ? color : Rgb{0, 0, 0};
    }
    return color;
}

void set_status_led(Rgb color) {
#if CONFIG_NS2_STATUS_LED_ENABLE
    if (s_board.led_strip == nullptr) {
        return;
    }
    if (color.r == s_board.last_color.r && color.g == s_board.last_color.g && color.b == s_board.last_color.b) {
        return;
    }
    s_board.last_color = color;
    led_strip_set_pixel(s_board.led_strip, 0, color.r, color.g, color.b);
    led_strip_refresh(s_board.led_strip);
#else
    (void)color;
#endif
}

void poll_status_led(int64_t now) {
    if (s_board.last_led_update_us != 0 && now - s_board.last_led_update_us < kLedUpdateUs) {
        return;
    }
    s_board.last_led_update_us = now;
    set_status_led(status_color(now));
}

} // namespace

esp_err_t board_controls_init() {
    gpio_config_t button_cfg = {};
    button_cfg.pin_bit_mask = 1ULL << CONFIG_NS2_BOOT_BUTTON_GPIO;
    button_cfg.mode = GPIO_MODE_INPUT;
    button_cfg.pull_up_en = GPIO_PULLUP_ENABLE;
    button_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    button_cfg.intr_type = GPIO_INTR_DISABLE;
    ESP_RETURN_ON_ERROR(gpio_config(&button_cfg), TAG, "BOOT button GPIO init failed");

    const int64_t now = esp_timer_get_time();
    s_board.button_level = gpio_get_level(static_cast<gpio_num_t>(CONFIG_NS2_BOOT_BUTTON_GPIO)) != 0;
    s_board.button_stable_level = s_board.button_level;
    s_board.button_changed_us = now;

#if CONFIG_NS2_STATUS_LED_ENABLE
    led_strip_config_t strip_config = {};
    strip_config.strip_gpio_num = CONFIG_NS2_STATUS_LED_GPIO;
    strip_config.max_leds = 1;
    strip_config.led_pixel_format = LED_PIXEL_FORMAT_GRB;
    strip_config.led_model = LED_MODEL_WS2812;
    led_strip_rmt_config_t rmt_config = {};
    rmt_config.resolution_hz = 10 * 1000 * 1000;
    rmt_config.flags.with_dma = false;
    ESP_RETURN_ON_ERROR(led_strip_new_rmt_device(&strip_config, &rmt_config, &s_board.led_strip),
                        TAG,
                        "status LED init failed");
    led_strip_clear(s_board.led_strip);
#endif

    s_board.initialized = true;
    ESP_LOGI(TAG, "board controls ready: boot_gpio=%d led_gpio=%d",
             CONFIG_NS2_BOOT_BUTTON_GPIO,
#if CONFIG_NS2_STATUS_LED_ENABLE
             CONFIG_NS2_STATUS_LED_GPIO
#else
             -1
#endif
    );
    return ESP_OK;
}

void board_controls_task() {
    if (!s_board.initialized) {
        return;
    }
    const int64_t now = esp_timer_get_time();
    poll_boot_button(now);
    poll_status_led(now);
}

} // namespace ns2
