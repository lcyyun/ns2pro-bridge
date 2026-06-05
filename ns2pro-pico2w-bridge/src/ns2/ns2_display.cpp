#include "ns2_display.h"

#include <cstdint>
#include <cstdio>
#include <cstring>

#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "ns2_input.h"
#include "ns2_state.h"
#include "ns2_status.h"
#include "ns2_usb.h"
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "tusb.h"

namespace {

constexpr uint16_t TFT_WIDTH = 240;
constexpr uint16_t TFT_HEIGHT = 240;
constexpr uint32_t TFT_SPI_BAUD = 8000000;
constexpr uint64_t REFRESH_US = 250000;
constexpr uint64_t DIAGNOSTIC_MODE_US = 4000000;
constexpr uint8_t SELECTED_PANEL_MODE = 5; // MODE 6: ST7789, SPI mode 3.
constexpr size_t SPI_SERVICE_CHUNK = 1024;

constexpr bool TFT_USE_CS = false;
constexpr uint8_t PIN_CS = 17;
constexpr uint8_t PIN_SCK = 18;
constexpr uint8_t PIN_MOSI = 19;
constexpr uint8_t PIN_DC = 20;
constexpr uint8_t PIN_RST = 21;

spi_inst_t *const TFT_SPI = spi0;

enum class PanelDriver : uint8_t {
    St7789 = 0,
    Gc9a01
};

struct PanelMode {
    const char *name;
    PanelDriver driver;
    uint16_t x_offset;
    uint16_t y_offset;
    uint8_t madctl;
    bool inverted;
    bool spi_mode_3;
    uint16_t color;
};

constexpr uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b) {
    return static_cast<uint16_t>(((r & 0xf8) << 8) | ((g & 0xfc) << 3) | (b >> 3));
}

constexpr uint16_t COLOR_BG = rgb565(8, 12, 18);
constexpr uint16_t COLOR_PANEL = rgb565(18, 26, 36);
constexpr uint16_t COLOR_TEXT = rgb565(232, 240, 248);
constexpr uint16_t COLOR_MUTED = rgb565(112, 128, 144);
constexpr uint16_t COLOR_GREEN = rgb565(41, 214, 137);
constexpr uint16_t COLOR_CYAN = rgb565(58, 190, 255);
constexpr uint16_t COLOR_YELLOW = rgb565(245, 196, 68);
constexpr uint16_t COLOR_ORANGE = rgb565(255, 138, 61);
constexpr uint16_t COLOR_RED = rgb565(255, 84, 84);
constexpr uint16_t COLOR_BLUE = rgb565(105, 146, 255);

struct DisplayRuntime {
    bool present;
    bool enabled;
    bool diagnostic;
    uint8_t mode_index;
    uint16_t x_offset;
    uint16_t y_offset;
    uint8_t madctl;
    bool inverted;
    bool spi_mode_3;
    uint64_t next_refresh_us;
    uint64_t next_mode_us;
    uint16_t frame[TFT_WIDTH * TFT_HEIGHT];
};

struct Glyph {
    char ch;
    uint8_t cols[5];
};

DisplayRuntime display{};

const PanelMode PANEL_MODES[] = {
    {"ST Y0 INV", PanelDriver::St7789, 0, 0, 0x00, true, false, COLOR_BLUE},
    {"ST Y80 INV", PanelDriver::St7789, 0, 80, 0x00, true, false, COLOR_GREEN},
    {"ST Y0 NOR", PanelDriver::St7789, 0, 0, 0x00, false, false, COLOR_YELLOW},
    {"ST Y80 NOR", PanelDriver::St7789, 0, 80, 0x00, false, false, COLOR_ORANGE},
    {"ST ROT INV", PanelDriver::St7789, 0, 0, 0x60, true, false, COLOR_CYAN},
    {"ST MODE3", PanelDriver::St7789, 0, 0, 0x00, true, true, COLOR_RED},
    {"GC9A01", PanelDriver::Gc9a01, 0, 0, 0x00, true, false, rgb565(180, 96, 255)},
};

const Glyph GLYPHS[] = {
    {' ', {0x00, 0x00, 0x00, 0x00, 0x00}},
    {'!', {0x00, 0x00, 0x5f, 0x00, 0x00}},
    {'?', {0x02, 0x01, 0x51, 0x09, 0x06}},
    {'.', {0x00, 0x60, 0x60, 0x00, 0x00}},
    {':', {0x00, 0x36, 0x36, 0x00, 0x00}},
    {'-', {0x08, 0x08, 0x08, 0x08, 0x08}},
    {'_', {0x40, 0x40, 0x40, 0x40, 0x40}},
    {'/', {0x20, 0x10, 0x08, 0x04, 0x02}},
    {'+', {0x08, 0x08, 0x3e, 0x08, 0x08}},
    {'0', {0x3e, 0x51, 0x49, 0x45, 0x3e}},
    {'1', {0x00, 0x42, 0x7f, 0x40, 0x00}},
    {'2', {0x42, 0x61, 0x51, 0x49, 0x46}},
    {'3', {0x21, 0x41, 0x45, 0x4b, 0x31}},
    {'4', {0x18, 0x14, 0x12, 0x7f, 0x10}},
    {'5', {0x27, 0x45, 0x45, 0x45, 0x39}},
    {'6', {0x3c, 0x4a, 0x49, 0x49, 0x30}},
    {'7', {0x01, 0x71, 0x09, 0x05, 0x03}},
    {'8', {0x36, 0x49, 0x49, 0x49, 0x36}},
    {'9', {0x06, 0x49, 0x49, 0x29, 0x1e}},
    {'A', {0x7e, 0x11, 0x11, 0x11, 0x7e}},
    {'B', {0x7f, 0x49, 0x49, 0x49, 0x36}},
    {'C', {0x3e, 0x41, 0x41, 0x41, 0x22}},
    {'D', {0x7f, 0x41, 0x41, 0x22, 0x1c}},
    {'E', {0x7f, 0x49, 0x49, 0x49, 0x41}},
    {'F', {0x7f, 0x09, 0x09, 0x09, 0x01}},
    {'G', {0x3e, 0x41, 0x49, 0x49, 0x7a}},
    {'H', {0x7f, 0x08, 0x08, 0x08, 0x7f}},
    {'I', {0x00, 0x41, 0x7f, 0x41, 0x00}},
    {'J', {0x20, 0x40, 0x41, 0x3f, 0x01}},
    {'K', {0x7f, 0x08, 0x14, 0x22, 0x41}},
    {'L', {0x7f, 0x40, 0x40, 0x40, 0x40}},
    {'M', {0x7f, 0x02, 0x0c, 0x02, 0x7f}},
    {'N', {0x7f, 0x04, 0x08, 0x10, 0x7f}},
    {'O', {0x3e, 0x41, 0x41, 0x41, 0x3e}},
    {'P', {0x7f, 0x09, 0x09, 0x09, 0x06}},
    {'Q', {0x3e, 0x41, 0x51, 0x21, 0x5e}},
    {'R', {0x7f, 0x09, 0x19, 0x29, 0x46}},
    {'S', {0x46, 0x49, 0x49, 0x49, 0x31}},
    {'T', {0x01, 0x01, 0x7f, 0x01, 0x01}},
    {'U', {0x3f, 0x40, 0x40, 0x40, 0x3f}},
    {'V', {0x1f, 0x20, 0x40, 0x20, 0x1f}},
    {'W', {0x3f, 0x40, 0x38, 0x40, 0x3f}},
    {'X', {0x63, 0x14, 0x08, 0x14, 0x63}},
    {'Y', {0x07, 0x08, 0x70, 0x08, 0x07}},
    {'Z', {0x61, 0x51, 0x49, 0x45, 0x43}},
};

const uint8_t *glyph_for(char ch) {
    if (ch >= 'a' && ch <= 'z') {
        ch = static_cast<char>(ch - 'a' + 'A');
    }
    for (const auto &glyph : GLYPHS) {
        if (glyph.ch == ch) {
            return glyph.cols;
        }
    }
    return glyph_for('?');
}

void select_display() {
    if (TFT_USE_CS) {
        gpio_put(PIN_CS, 0);
    }
}

void unselect_display() {
    if (TFT_USE_CS) {
        gpio_put(PIN_CS, 1);
    }
}

void write_spi(const uint8_t *data, size_t len) {
    if (!data || len == 0) {
        return;
    }
    size_t offset = 0;
    while (offset < len) {
        const size_t chunk = len - offset > SPI_SERVICE_CHUNK ? SPI_SERVICE_CHUNK : len - offset;
        spi_write_blocking(TFT_SPI, data + offset, chunk);
        offset += chunk;
        if (len > SPI_SERVICE_CHUNK) {
            cyw43_arch_poll();
            tud_task();
            ns2_usb_task();
        }
    }
}

void write_command(uint8_t command) {
    select_display();
    gpio_put(PIN_DC, 0);
    write_spi(&command, 1);
    unselect_display();
}

void write_data(const uint8_t *data, size_t len) {
    select_display();
    gpio_put(PIN_DC, 1);
    write_spi(data, len);
    unselect_display();
}

void command_data(uint8_t command, const uint8_t *data, size_t len) {
    select_display();
    gpio_put(PIN_DC, 0);
    write_spi(&command, 1);
    if (data && len > 0) {
        gpio_put(PIN_DC, 1);
        write_spi(data, len);
    }
    unselect_display();
}

void reset_panel() {
    gpio_put(PIN_RST, 1);
    sleep_ms(20);
    gpio_put(PIN_RST, 0);
    sleep_ms(30);
    gpio_put(PIN_RST, 1);
    sleep_ms(120);
}

void set_addr_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    x0 = static_cast<uint16_t>(x0 + display.x_offset);
    x1 = static_cast<uint16_t>(x1 + display.x_offset);
    y0 = static_cast<uint16_t>(y0 + display.y_offset);
    y1 = static_cast<uint16_t>(y1 + display.y_offset);

    const uint8_t col[] = {
        static_cast<uint8_t>(x0 >> 8), static_cast<uint8_t>(x0 & 0xff),
        static_cast<uint8_t>(x1 >> 8), static_cast<uint8_t>(x1 & 0xff),
    };
    const uint8_t row[] = {
        static_cast<uint8_t>(y0 >> 8), static_cast<uint8_t>(y0 & 0xff),
        static_cast<uint8_t>(y1 >> 8), static_cast<uint8_t>(y1 & 0xff),
    };
    command_data(0x2a, col, sizeof(col));
    command_data(0x2b, row, sizeof(row));
    write_command(0x2c);
}

void init_st7789_controller() {
    reset_panel();

    write_command(0x01);
    sleep_ms(150);
    write_command(0x11);
    sleep_ms(150);

    const uint8_t color_mode[] = {0x55};
    command_data(0x3a, color_mode, sizeof(color_mode));

    const uint8_t madctl[] = {display.madctl};
    command_data(0x36, madctl, sizeof(madctl));

    const uint8_t porch[] = {0x0c, 0x0c, 0x00, 0x33, 0x33};
    command_data(0xb2, porch, sizeof(porch));
    const uint8_t gate[] = {0x35};
    command_data(0xb7, gate, sizeof(gate));
    const uint8_t vcom[] = {0x19};
    command_data(0xbb, vcom, sizeof(vcom));
    const uint8_t lcm[] = {0x2c};
    command_data(0xc0, lcm, sizeof(lcm));
    const uint8_t vdv_vrh[] = {0x01};
    command_data(0xc2, vdv_vrh, sizeof(vdv_vrh));
    const uint8_t vrh[] = {0x12};
    command_data(0xc3, vrh, sizeof(vrh));
    const uint8_t vdv[] = {0x20};
    command_data(0xc4, vdv, sizeof(vdv));
    const uint8_t frame_rate[] = {0x0f};
    command_data(0xc6, frame_rate, sizeof(frame_rate));
    const uint8_t power[] = {0xa4, 0xa1};
    command_data(0xd0, power, sizeof(power));

    write_command(display.inverted ? 0x21 : 0x20);
    write_command(0x13);
    write_command(0x29);
    sleep_ms(120);
}

void init_gc9a01_controller() {
    reset_panel();

    write_command(0xef);
    const uint8_t eb[] = {0x14};
    command_data(0xeb, eb, sizeof(eb));
    write_command(0xfe);
    write_command(0xef);
    command_data(0xeb, eb, sizeof(eb));
    const uint8_t maddr[] = {display.madctl};
    command_data(0x36, maddr, sizeof(maddr));
    const uint8_t color_mode[] = {0x55};
    command_data(0x3a, color_mode, sizeof(color_mode));
    const uint8_t b6[] = {0x00, 0x20};
    command_data(0xb6, b6, sizeof(b6));
    const uint8_t c3[] = {0x13};
    command_data(0xc3, c3, sizeof(c3));
    const uint8_t c4[] = {0x13};
    command_data(0xc4, c4, sizeof(c4));
    const uint8_t c9[] = {0x22};
    command_data(0xc9, c9, sizeof(c9));
    const uint8_t c6[] = {0x0f};
    command_data(0xc6, c6, sizeof(c6));
    const uint8_t df[] = {0x21, 0x0c, 0x02};
    command_data(0xdf, df, sizeof(df));
    const uint8_t f0[] = {0x45, 0x09, 0x08, 0x08, 0x26, 0x2a};
    command_data(0xf0, f0, sizeof(f0));
    const uint8_t f1[] = {0x43, 0x70, 0x72, 0x36, 0x37, 0x6f};
    command_data(0xf1, f1, sizeof(f1));
    const uint8_t f2[] = {0x45, 0x09, 0x08, 0x08, 0x26, 0x2a};
    command_data(0xf2, f2, sizeof(f2));
    const uint8_t f3[] = {0x43, 0x70, 0x72, 0x36, 0x37, 0x6f};
    command_data(0xf3, f3, sizeof(f3));
    const uint8_t ed[] = {0x1b, 0x0b};
    command_data(0xed, ed, sizeof(ed));
    const uint8_t ae[] = {0x77};
    command_data(0xae, ae, sizeof(ae));
    const uint8_t cd[] = {0x63};
    command_data(0xcd, cd, sizeof(cd));

    write_command(display.inverted ? 0x21 : 0x20);
    write_command(0x11);
    sleep_ms(120);
    write_command(0x29);
    sleep_ms(120);
}

void apply_panel_mode(uint8_t index) {
    if (index >= sizeof(PANEL_MODES) / sizeof(PANEL_MODES[0])) {
        index = 0;
    }

    const PanelMode &mode = PANEL_MODES[index];
    display.mode_index = index;
    display.x_offset = mode.x_offset;
    display.y_offset = mode.y_offset;
    display.madctl = mode.madctl;
    display.inverted = mode.inverted;
    display.spi_mode_3 = mode.spi_mode_3;

    spi_set_format(TFT_SPI,
                   8,
                   mode.spi_mode_3 ? SPI_CPOL_1 : SPI_CPOL_0,
                   mode.spi_mode_3 ? SPI_CPHA_1 : SPI_CPHA_0,
                   SPI_MSB_FIRST);

    if (mode.driver == PanelDriver::Gc9a01) {
        init_gc9a01_controller();
    } else {
        init_st7789_controller();
    }

    printf("[NS2 DISP] test mode %u %s xoff=%u yoff=%u mad=0x%02x inv=%u spi=%u\n",
           static_cast<unsigned>(index + 1),
           mode.name,
           static_cast<unsigned>(mode.x_offset),
           static_cast<unsigned>(mode.y_offset),
           static_cast<unsigned>(mode.madctl),
           mode.inverted ? 1u : 0u,
           mode.spi_mode_3 ? 3u : 0u);
}

void clear_frame(uint16_t color) {
    for (size_t i = 0; i < TFT_WIDTH * TFT_HEIGHT; i++) {
        display.frame[i] = color;
    }
}

void set_pixel(int x, int y, uint16_t color) {
    if (x < 0 || y < 0 || x >= TFT_WIDTH || y >= TFT_HEIGHT) {
        return;
    }
    display.frame[static_cast<size_t>(y) * TFT_WIDTH + static_cast<size_t>(x)] = color;
}

void fill_rect(int x, int y, int w, int h, uint16_t color) {
    if (w <= 0 || h <= 0 || x >= TFT_WIDTH || y >= TFT_HEIGHT) {
        return;
    }
    if (x < 0) {
        w += x;
        x = 0;
    }
    if (y < 0) {
        h += y;
        y = 0;
    }
    if (x + w > TFT_WIDTH) {
        w = TFT_WIDTH - x;
    }
    if (y + h > TFT_HEIGHT) {
        h = TFT_HEIGHT - y;
    }
    if (w <= 0 || h <= 0) {
        return;
    }
    for (int row = 0; row < h; row++) {
        uint16_t *dst = display.frame + static_cast<size_t>(y + row) * TFT_WIDTH + static_cast<size_t>(x);
        for (int col = 0; col < w; col++) {
            dst[col] = color;
        }
    }
}

uint16_t text_width(const char *text, uint8_t scale) {
    if (!text) {
        return 0;
    }
    const size_t len = strlen(text);
    if (len == 0) {
        return 0;
    }
    return static_cast<uint16_t>((len * 6 - 1) * scale);
}

void draw_char(int x, int y, char ch, uint8_t scale, uint16_t color) {
    const uint8_t *glyph = glyph_for(ch);
    for (uint8_t col = 0; col < 5; col++) {
        for (uint8_t row = 0; row < 7; row++) {
            if ((glyph[col] & (1u << row)) == 0) {
                continue;
            }
            fill_rect(x + col * scale, y + row * scale, scale, scale, color);
        }
    }
}

void draw_text(int x, int y, const char *text, uint8_t scale, uint16_t color) {
    if (!text) {
        return;
    }
    while (*text && x < TFT_WIDTH) {
        draw_char(x, y, *text++, scale, color);
        x += 6 * scale;
    }
}

void draw_text_center(int y, const char *text, uint8_t scale, uint16_t color) {
    const uint16_t width = text_width(text, scale);
    int x = 0;
    if (width < TFT_WIDTH) {
        x = (TFT_WIDTH - width) / 2;
    }
    draw_text(x, y, text, scale, color);
}

const char *short_state(Ns2BleState state) {
    switch (state) {
        case Ns2BleState::Boot:
            return "BOOT";
        case Ns2BleState::BleInit:
            return "BLE INIT";
        case Ns2BleState::Idle:
            return "IDLE";
        case Ns2BleState::Scanning:
            return "SCAN";
        case Ns2BleState::Connecting:
            return "CONNECTING";
        case Ns2BleState::Pairing:
            return "PAIRING";
        case Ns2BleState::Discovering:
            return "GATT";
        case Ns2BleState::Subscribing:
            return "SUBSCRIBE";
        case Ns2BleState::InitializingController:
            return "INIT PAD";
        case Ns2BleState::ConnectedNoNotify:
            return "NO INPUT";
        case Ns2BleState::ConnectedLive:
            return "LIVE";
        case Ns2BleState::Disconnected:
            return "DISCONN";
        case Ns2BleState::Backoff:
            return "BACKOFF";
        case Ns2BleState::Error:
            return "ERROR";
        default:
            return "UNKNOWN";
    }
}

uint16_t status_color(Ns2BleState state) {
    switch (state) {
        case Ns2BleState::ConnectedLive:
            return COLOR_GREEN;
        case Ns2BleState::ConnectedNoNotify:
        case Ns2BleState::Subscribing:
        case Ns2BleState::InitializingController:
            return COLOR_CYAN;
        case Ns2BleState::Connecting:
        case Ns2BleState::Pairing:
        case Ns2BleState::Discovering:
            return COLOR_BLUE;
        case Ns2BleState::Scanning:
            return COLOR_YELLOW;
        case Ns2BleState::Backoff:
        case Ns2BleState::Disconnected:
            return COLOR_ORANGE;
        case Ns2BleState::Error:
            return COLOR_RED;
        default:
            return COLOR_MUTED;
    }
}

const char *state_hint(Ns2BleState state) {
    switch (state) {
        case Ns2BleState::Scanning:
            return "PAIR MODE";
        case Ns2BleState::Connecting:
            return "WAKE PAD";
        case Ns2BleState::Pairing:
            return "BONDING";
        case Ns2BleState::Discovering:
            return "READING GATT";
        case Ns2BleState::Subscribing:
            return "SUBSCRIBE";
        case Ns2BleState::InitializingController:
            return "INIT CMDS";
        case Ns2BleState::ConnectedNoNotify:
            return "WAIT INPUT";
        case Ns2BleState::ConnectedLive:
            return "INPUT OK";
        case Ns2BleState::Backoff:
            return "RETRY WAIT";
        case Ns2BleState::Disconnected:
            return "DISCONNECTED";
        case Ns2BleState::Idle:
            return "READY";
        case Ns2BleState::Boot:
        case Ns2BleState::BleInit:
            return "STARTING";
        case Ns2BleState::Error:
            return "CHECK SERIAL";
        default:
            return "";
    }
}

void draw_status_panel() {
    const Ns2BleState state = ns2_status_get_state();
    const uint16_t accent = status_color(state);
    char line[48];
    Ns2InputSnapshot input;
    Ns2MotionSample motion;
    Ns2UsbStats usb_stats;
    ns2_usb_get_stats(&usb_stats);
    const bool show_input = ns2_input_get_snapshot(&input) &&
                            (state == Ns2BleState::ConnectedLive ||
                             ns2_status_notify_count() > 0);

    clear_frame(COLOR_BG);
    fill_rect(0, 0, TFT_WIDTH, 48, COLOR_PANEL);
    fill_rect(0, 48, TFT_WIDTH, 4, accent);

    if (show_input) {
        const bool motion_valid = ns2_input_get_motion_sample(&motion);

        draw_text_center(8, short_state(state), 3, accent);

        snprintf(line,
                 sizeof(line),
                 "%s %uB %luHZ",
                 ns2_input_kind_name(input.kind),
                 static_cast<unsigned>(input.len),
                 static_cast<unsigned long>(ns2_status_notify_hz()));
        draw_text_center(38, line, 2, COLOR_TEXT);

        snprintf(line,
                 sizeof(line),
                 "BTN %08lX",
                 static_cast<unsigned long>(input.buttons));
        draw_text_center(62, line, 2, input.buttons ? COLOR_GREEN : COLOR_MUTED);

        char buttons[48];
        ns2_input_format_buttons(input.buttons, buttons, sizeof(buttons));
        draw_text_center(84, buttons, strlen(buttons) > 19 ? 1 : 2, input.buttons ? COLOR_GREEN : COLOR_MUTED);

        snprintf(line,
                 sizeof(line),
                 "L%4u %4u R%4u %4u",
                 static_cast<unsigned>(input.lx),
                 static_cast<unsigned>(input.ly),
                 static_cast<unsigned>(input.rx),
                 static_cast<unsigned>(input.ry));
        draw_text_center(110, line, 1, COLOR_TEXT);

        if (motion_valid) {
            snprintf(line,
                     sizeof(line),
                     "ACC %d %d %d",
                     static_cast<int>(motion.accel[0]),
                     static_cast<int>(motion.accel[1]),
                     static_cast<int>(motion.accel[2]));
            draw_text_center(132, line, 1, COLOR_CYAN);

            snprintf(line,
                     sizeof(line),
                     "GYR %d %d %d",
                     static_cast<int>(motion.gyro[0]),
                     static_cast<int>(motion.gyro[1]),
                     static_cast<int>(motion.gyro[2]));
            draw_text_center(152, line, 1, COLOR_CYAN);
        } else {
            draw_text_center(142, "MOTION -", 2, COLOR_MUTED);
        }

        snprintf(line,
                 sizeof(line),
                 "UPD %lu MOT O%u #%lu USB %s",
                 static_cast<unsigned long>(input.updates),
                 static_cast<unsigned>(input.motion_offset),
                 static_cast<unsigned long>(input.motion_updates),
                 usb_stats.suspended ? "S" : usb_stats.mounted ? "M" : "-");
        draw_text_center(178, line, 1, usb_stats.mounted && usb_stats.reports_failed == 0 ? COLOR_TEXT : COLOR_MUTED);

        snprintf(line,
                 sizeof(line),
                 "RUM %lu/%lu E%lu",
                 static_cast<unsigned long>(usb_stats.rumble_writes),
                 static_cast<unsigned long>(usb_stats.rumble_updates),
                 static_cast<unsigned long>(usb_stats.rumble_errors));
        draw_text_center(204, line, 1, usb_stats.rumble_writes > 0 ? COLOR_GREEN : COLOR_MUTED);
        return;
    }

    draw_text_center(20, short_state(state), 4, accent);
    draw_text_center(84, state_hint(state), 2, COLOR_TEXT);

    snprintf(line,
             sizeof(line),
             "AUTO %s   SAVED %s",
             ns2_config_auto_connect_enabled() ? "ON" : "OFF",
             ns2_config_has_saved_target() ? "YES" : "NO");
    draw_text_center(116, line, 2, COLOR_MUTED);

    snprintf(line,
             sizeof(line),
             "TRY %lu   DROP %lu",
             static_cast<unsigned long>(ns2_status_connect_attempts()),
             static_cast<unsigned long>(ns2_status_disconnect_count()));
    draw_text_center(144, line, 2, COLOR_TEXT);

    if (state == Ns2BleState::ConnectedLive || ns2_status_notify_count() > 0) {
        snprintf(line,
                 sizeof(line),
                 "INPUT %lu  %luHZ",
                 static_cast<unsigned long>(ns2_status_notify_count()),
                 static_cast<unsigned long>(ns2_status_notify_hz()));
        draw_text_center(172, line, 2, COLOR_GREEN);
    } else {
        const char *error = ns2_status_last_error();
        snprintf(line, sizeof(line), "ERR %s", error && error[0] ? error : "-");
        draw_text_center(172, line, 2, error && error[0] ? COLOR_ORANGE : COLOR_MUTED);
    }

    const uint32_t age_ms = ns2_status_last_notify_age_ms();
    if (age_ms == UINT32_MAX) {
        snprintf(line, sizeof(line), "RSSI %d  AGE -", ns2_status_rssi());
    } else {
        snprintf(line,
                 sizeof(line),
                 "RSSI %d  AGE %lus",
                 ns2_status_rssi(),
                 static_cast<unsigned long>((age_ms + 500) / 1000));
    }
    draw_text_center(204, line, 2, COLOR_MUTED);
}

void draw_diagnostic_panel() {
    const PanelMode &mode = PANEL_MODES[display.mode_index];
    char line[32];

    clear_frame(mode.color);
    fill_rect(0, 0, TFT_WIDTH, 64, COLOR_BG);
    fill_rect(0, 184, TFT_WIDTH, 56, COLOR_BG);

    snprintf(line, sizeof(line), "MODE %u", static_cast<unsigned>(display.mode_index + 1));
    draw_text_center(20, line, 4, COLOR_TEXT);
    draw_text_center(92, mode.name, 3, COLOR_BG);

    snprintf(line,
             sizeof(line),
             "X%u Y%u M%02X",
             static_cast<unsigned>(mode.x_offset),
             static_cast<unsigned>(mode.y_offset),
             static_cast<unsigned>(mode.madctl));
    draw_text_center(136, line, 2, COLOR_BG);

    snprintf(line,
             sizeof(line),
             "INV %c SPI %u",
             mode.inverted ? 'Y' : 'N',
             mode.spi_mode_3 ? 3u : 0u);
    draw_text_center(196, line, 2, COLOR_TEXT);
}

void flush_frame() {
    set_addr_window(0, 0, TFT_WIDTH - 1, TFT_HEIGHT - 1);

    uint8_t tx[1024];
    size_t pixel = 0;
    const size_t total_pixels = TFT_WIDTH * TFT_HEIGHT;

    select_display();
    gpio_put(PIN_DC, 1);
    while (pixel < total_pixels) {
        const size_t remaining = total_pixels - pixel;
        const size_t chunk_pixels = remaining > sizeof(tx) / 2 ? sizeof(tx) / 2 : remaining;
        for (size_t i = 0; i < chunk_pixels; i++) {
            const uint16_t color = display.frame[pixel + i];
            tx[i * 2] = static_cast<uint8_t>(color >> 8);
            tx[i * 2 + 1] = static_cast<uint8_t>(color & 0xff);
        }
        write_spi(tx, chunk_pixels * 2);
        pixel += chunk_pixels;
    }
    unselect_display();
}

} // namespace

void ns2_display_init() {
    memset(&display, 0, sizeof(display));
    display.enabled = ns2_config_display_enabled();
    if (!display.enabled) {
        printf("[NS2 DISP] disabled by config\n");
        return;
    }
    display.diagnostic = false;

    spi_init(TFT_SPI, TFT_SPI_BAUD);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    if (TFT_USE_CS) {
        gpio_init(PIN_CS);
        gpio_set_dir(PIN_CS, GPIO_OUT);
        gpio_put(PIN_CS, 1);
    }
    gpio_init(PIN_DC);
    gpio_set_dir(PIN_DC, GPIO_OUT);
    gpio_put(PIN_DC, 1);
    gpio_init(PIN_RST);
    gpio_set_dir(PIN_RST, GPIO_OUT);
    gpio_put(PIN_RST, 1);

    apply_panel_mode(SELECTED_PANEL_MODE);
    display.present = true;
    display.next_mode_us = 0;
    draw_status_panel();
    flush_frame();

    printf("[NS2 DISP] ST7789 final mode 6 240x240 spi0 sck=GP%u mosi=GP%u cs=%s dc=GP%u rst=GP%u baud=%lu\n",
           static_cast<unsigned>(PIN_SCK),
           static_cast<unsigned>(PIN_MOSI),
           TFT_USE_CS ? "GP17" : "none",
           static_cast<unsigned>(PIN_DC),
           static_cast<unsigned>(PIN_RST),
           static_cast<unsigned long>(TFT_SPI_BAUD));
}

void ns2_display_tick() {
    if (!display.enabled || !display.present) {
        return;
    }

    const uint64_t now = time_us_64();
    if (display.next_refresh_us != 0 && now < display.next_refresh_us) {
        return;
    }
    display.next_refresh_us = now + REFRESH_US;

    if (display.diagnostic) {
        if (display.next_mode_us != 0 && now >= display.next_mode_us) {
            const uint8_t next = static_cast<uint8_t>((display.mode_index + 1) %
                (sizeof(PANEL_MODES) / sizeof(PANEL_MODES[0])));
            apply_panel_mode(next);
            display.next_mode_us = now + DIAGNOSTIC_MODE_US;
        }
        draw_diagnostic_panel();
    } else {
        draw_status_panel();
    }
    flush_frame();
}

void ns2_display_set_enabled(bool enabled) {
    ns2_config_set_display_enabled(enabled);
    if (enabled) {
        if (!display.present) {
            ns2_display_init();
        }
        display.enabled = true;
        return;
    }

    display.enabled = false;
    if (display.present) {
        write_command(0x28);
    }
}

bool ns2_display_enabled() {
    return display.enabled;
}
