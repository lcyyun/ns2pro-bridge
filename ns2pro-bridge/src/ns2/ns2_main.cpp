#include <cstdio>
#include <cstring>

#include "bsp/board_api.h"
#include "hardware/clocks.h"
#include "hardware/vreg.h"
#include "ns2_ble.h"
#include "ns2_display.h"
#include "ns2_input.h"
#include "ns2_state.h"
#include "ns2_status.h"
#include "ns2_usb.h"
#include "pico/bootrom.h"
#include "pico/cyw43_arch.h"
#include "pico/error.h"
#include "pico/stdio.h"
#include "pico/stdlib.h"
#include "tusb.h"

namespace {

char line_buffer[128];
size_t line_len = 0;

const char *skip_spaces(const char *s) {
    while (*s == ' ' || *s == '\t') {
        s++;
    }
    return s;
}

bool arg_is(const char *line, const char *prefix) {
    const size_t len = strlen(prefix);
    return strncmp(line, prefix, len) == 0 && (line[len] == 0 || line[len] == ' ' || line[len] == '\t');
}

void print_status() {
    char json[512];
    ns2_status_format_json(json, sizeof(json));
    printf("%s\n", json);
}

void print_candidates() {
    char json[768];
    ns2_status_format_candidates_json(json, sizeof(json));
    printf("%s\n", json);
}

void print_motion_status() {
    Ns2InputSnapshot input;
    const bool valid = ns2_input_get_snapshot(&input);
    char motion_json[384];
    ns2_input_format_motion_json(valid ? &input : nullptr, motion_json, sizeof(motion_json));
    char json[768];
    snprintf(json,
             sizeof(json),
             "{\"ok\":true,\"profile\":\"ns2pro\",\"input_valid\":%s,"
             "\"kind\":\"%s\",\"len\":%u,\"updates\":%lu,"
             "\"buttons\":%lu,\"lx\":%u,\"ly\":%u,\"rx\":%u,\"ry\":%u,%s}",
             valid ? "true" : "false",
             valid ? ns2_input_kind_name(input.kind) : "UNK",
             valid ? static_cast<unsigned>(input.len) : 0u,
             valid ? static_cast<unsigned long>(input.updates) : 0ul,
             valid ? static_cast<unsigned long>(input.buttons) : 0ul,
             valid ? static_cast<unsigned>(input.lx) : 2048u,
             valid ? static_cast<unsigned>(input.ly) : 2048u,
             valid ? static_cast<unsigned>(input.rx) : 2048u,
             valid ? static_cast<unsigned>(input.ry) : 2048u,
             motion_json);
    printf("%s\n", json);
}

void print_help() {
    printf("commands: status | usb status | usb rate <hz> | usb raw on|off | motion status | rumble config|tune|test|stop | settings status|save | display on|off | web parse on|off | ns2 scan | ns2 pair | ns2 reconnect | ns2 disconnect | ns2 forget | ns2 auto on|off | ns2 candidates | bootrom\n");
}

void handle_command(const char *raw_line) {
    const char *line = skip_spaces(raw_line);
    if (line[0] == 0) {
        return;
    }

    if (strcmp(line, "status") == 0 || strcmp(line, "ns2 status") == 0) {
        print_status();
        return;
    }
    if (strcmp(line, "usb status") == 0) {
        Ns2UsbStats stats;
        ns2_usb_get_stats(&stats);
        printf("{\"ok\":true,\"profile\":\"ns2pro\",\"usb_mounted\":%s,\"usb_suspended\":%s,"
               "\"reports_sent\":%lu,\"reports_failed\":%lu,"
               "\"raw_passthrough_reports\":%lu,\"parsed_reports\":%lu,\"hid_out\":%lu,"
               "\"report_rate_hz\":%u,\"report_interval_us\":%lu,"
               "\"hid_last_report\":\"0x%02x\",\"vendor_out\":%lu,\"vendor_in\":%lu,"
               "\"rumble_active\":%s,\"rumble_updates\":%lu,\"rumble_writes\":%lu,"
               "\"rumble_stops\":%lu,\"rumble_errors\":%lu,"
               "\"rumble_enabled\":%s,\"rumble_scale_percent\":%u,"
               "\"rumble_hold_ms\":%u,\"rumble_tick_ms\":%u,"
               "\"rumble_stop_packets\":%u,\"display_enabled\":%s,"
               "\"usb_raw_passthrough\":%s,\"web_parse_reports\":%s,"
               "\"feature_set\":%lu,\"feature_get\":%lu}\n",
               stats.mounted ? "true" : "false",
               stats.suspended ? "true" : "false",
               static_cast<unsigned long>(stats.reports_sent),
               static_cast<unsigned long>(stats.reports_failed),
               static_cast<unsigned long>(stats.raw_passthrough_reports),
               static_cast<unsigned long>(stats.parsed_reports),
               static_cast<unsigned long>(stats.hid_out_count),
               static_cast<unsigned>(stats.report_rate_hz),
               static_cast<unsigned long>(stats.report_interval_us),
               stats.hid_last_effective_report_id,
               static_cast<unsigned long>(stats.vendor_out_count),
               static_cast<unsigned long>(stats.vendor_in_count),
               stats.rumble_active ? "true" : "false",
               static_cast<unsigned long>(stats.rumble_updates),
               static_cast<unsigned long>(stats.rumble_writes),
               static_cast<unsigned long>(stats.rumble_stops),
               static_cast<unsigned long>(stats.rumble_errors),
               stats.rumble_enabled ? "true" : "false",
               static_cast<unsigned>(stats.rumble_scale_percent),
               static_cast<unsigned>(stats.rumble_hold_ms),
               static_cast<unsigned>(stats.rumble_tick_ms),
               static_cast<unsigned>(stats.rumble_stop_packets),
               ns2_config_display_enabled() ? "true" : "false",
               ns2_config_usb_raw_passthrough_enabled() ? "true" : "false",
               ns2_config_web_parse_reports_enabled() ? "true" : "false",
               static_cast<unsigned long>(stats.feature_set_count),
               static_cast<unsigned long>(stats.feature_get_count));
        return;
    }
    if (strcmp(line, "motion status") == 0 || strcmp(line, "imu status") == 0) {
        print_motion_status();
        return;
    }
    if (arg_is(line, "rumble") || arg_is(line, "usb rate") || arg_is(line, "usb raw") ||
        arg_is(line, "report rate") || arg_is(line, "settings") || arg_is(line, "config") ||
        arg_is(line, "display") || arg_is(line, "screen") || arg_is(line, "web parse") ||
        arg_is(line, "webui parse") || strcmp(line, "usb config") == 0 ||
        strcmp(line, "report config") == 0) {
        char json[512];
        if (ns2_usb_handle_debug_command(line, json, sizeof(json))) {
            printf("%s\n", json);
        }
        return;
    }
    if (strcmp(line, "ns2 scan") == 0) {
        ns2_ble_start_scan();
        print_status();
        return;
    }
    if (strcmp(line, "ns2 pair") == 0) {
        ns2_ble_pair();
        print_status();
        return;
    }
    if (strcmp(line, "ns2 reconnect") == 0) {
        ns2_ble_reconnect();
        print_status();
        return;
    }
    if (strcmp(line, "ns2 disconnect") == 0) {
        ns2_ble_disconnect();
        print_status();
        return;
    }
    if (strcmp(line, "ns2 forget") == 0) {
        ns2_ble_forget();
        print_status();
        return;
    }
    if (strcmp(line, "ns2 candidates") == 0) {
        print_candidates();
        return;
    }
    if (arg_is(line, "ns2 auto")) {
        const char *arg = skip_spaces(line + strlen("ns2 auto"));
        if (strcmp(arg, "on") == 0) {
            ns2_ble_set_auto_connect(true);
            print_status();
            return;
        }
        if (strcmp(arg, "off") == 0) {
            ns2_ble_set_auto_connect(false);
            print_status();
            return;
        }
    }
    if (strcmp(line, "help") == 0 || strcmp(line, "?") == 0) {
        print_help();
        return;
    }
    if (strcmp(line, "bootrom") == 0 || strcmp(line, "uf2") == 0) {
        printf("{\"ok\":true,\"action\":\"bootrom\"}\n");
        sleep_ms(100);
        reset_usb_boot(0, 0);
    }

    printf("{\"ok\":false,\"error\":\"unknown command\"}\n");
}

void serial_poll() {
    while (true) {
        const int ch = getchar_timeout_us(0);
        if (ch == PICO_ERROR_TIMEOUT) {
            return;
        }

        if (ch == '\r' || ch == '\n') {
            if (line_len > 0) {
                line_buffer[line_len] = 0;
                handle_command(line_buffer);
                line_len = 0;
            }
            continue;
        }

        if (ch == 0x08 || ch == 0x7f) {
            if (line_len > 0) {
                line_len--;
            }
            continue;
        }

        if (line_len + 1 < sizeof(line_buffer)) {
            line_buffer[line_len++] = static_cast<char>(ch);
        }
    }
}

} // namespace

int main() {
    vreg_set_voltage(VREG_VOLTAGE_1_20);
    sleep_ms(1000);
    set_sys_clock_khz(SYS_CLOCK_KHZ, true);

    board_init();
    tusb_rhport_init_t dev_init = {
        .role = TUSB_ROLE_DEVICE,
        .speed = TUSB_SPEED_FULL,
    };
    tusb_init(BOARD_TUD_RHPORT, &dev_init);
    board_init_after_tusb();
    stdio_init_all();

    if (cyw43_arch_init()) {
        printf("[NS2] failed to initialize CYW43\n");
        return 1;
    }

    ns2_status_init();
    ns2_config_load();
    ns2_usb_init();
    ns2_display_init();
    printf("[NS2] Pico 2 W NS2 Pro BLE auto-connect MVP boot\n");
    print_help();

    ns2_ble_init();

    while (true) {
        cyw43_arch_poll();
        tud_task();
        ns2_ble_tick();
        ns2_usb_task();
        ns2_status_tick_led();
        ns2_display_tick();
        serial_poll();
        sleep_ms(1);
    }
}
