# NS2Pro Pico 2 W Bridge Project Notes

Date: 2026-06-05
Status: Active project notes
Project name: `ns2pro-bridge`
Target hardware: Raspberry Pi Pico 2 W

## 1. Project Goal

This project bridges one NS2Pro controller to a host over USB:

```text
NS2Pro controller -> BLE -> Pico 2 W -> Nintendo-style USB HID -> host
```

The project is not trying to become a universal controller framework yet. The
current priority is a stable single-controller firmware with a useful WebHID
tuner and enough documentation to continue development safely.

## 2. Current Decision Summary

These decisions are considered current unless later tests prove otherwise:

- Keep the BLE connection path simple and stable.
- Treat long-press pairing/manual wake as the reliable connection path for now.
- Do not spend more time on complex automatic reconnect experiments until better
  NS2Pro-specific information is available.
- Use parsed FD2 input and repack it into Nintendo-style USB reports.
- Keep raw FD2-to-USB passthrough disabled by default.
- Treat raw passthrough as diagnostic only.
- Keep the ST7789 display disabled by default because high-frequency display
  refresh can affect runtime behavior.
- Use WebHID as the main configuration and tuning interface.
- Keep serial debug logs available for development.

## 3. Implemented Features

### BLE and Controller Initialization

- Scans for NS2Pro BLE controllers.
- Connects to a matching controller.
- Runs the GATT discovery and controller initialization sequence.
- Subscribes to input notifications.
- Tracks connection state, notification counters, and recent errors.

Known limitation:

- Automatic reconnect is not considered solved. The current firmware should
  preserve the stable connection flow, but reconnect behavior may still require
  controller wake/pairing-button interaction depending on controller state.

### FD2 Input Parsing

Current FD2 parsing:

- buttons from `data[4..7]`;
- left stick from packed 12-bit data at `data[10..12]`;
- right stick from packed 12-bit data at `data[13..15]`;
- accel and gyro from the motion block starting at `data[48]`.

Current interpretation:

- These offsets match the tested reference implementation used during
  development.
- The stick boundary issue is more likely caused by wireless-vs-USB range
  semantics and output mapping, not by a simple FD2 offset error.

### USB HID Output

The stable output path is:

```text
BLE FD2 input -> parse -> center/deadzone handling -> range mapping -> USB report
```

Raw USB passthrough path:

```text
BLE FD2 input -> copied into USB report body
```

Raw passthrough is kept only for diagnostics. It may not match the controller's
native USB behavior and can produce incorrect stick outer-boundary results.

### WebHID Tuner

The WebHID tuner currently supports:

- connect / use existing HID device;
- English and Chinese UI;
- connection, input, USB, and rumble status;
- live stick and motion visualization;
- 3D cube motion visualization;
- target and actual report-rate display;
- rumble parameter tuning;
- display enable/disable;
- raw USB diagnostic enable/disable;
- settings apply and flash save.

Expected settings behavior:

- `Apply` changes runtime settings for the current boot.
- `Save` writes settings to flash.
- Defaults should be conservative and stable.

### ST7789 Display

The firmware supports an optional ST7789 240x240 SPI display.

Current decision:

- Display is off by default.
- It is useful for basic status, but high-frequency updates are not worth the
  performance cost.
- Future display work should prefer compact, low-rate status updates.

## 4. Current Defaults

Default runtime settings:

```text
display_enabled = false
usb_raw_passthrough = false
web_parse_reports = true
rumble_enabled = true
report_rate_hz = 250
```

Default rumble parameters:

```text
scale_percent = 60
hold_ms = 140
tick_ms = 30
stop_packets = 3
```

## 5. Recommended Test Method

### Basic Connection Test

1. Flash the current UF2 to Pico 2 W.
2. Power the NS2Pro controller and use the known reliable pairing/wake flow.
3. Confirm the WebUI reports `Connected`.
4. Confirm input reports are increasing.
5. Test all buttons and sticks.

### USB Output Test

1. Open the WebHID tuner.
2. Keep `Raw USB (Diag)` off.
3. Click `Apply`.
4. Confirm parsed reports are increasing.
5. Test the controller in a game or controller tester.
6. Save settings only after the behavior is known good.

### Stick Boundary Test

1. Keep raw passthrough off.
2. Slowly rotate each stick around the outer edge.
3. Check that the reported outer boundary reaches the expected range.
4. Check in-game behavior, especially whether full tilt always produces
   full-speed movement.
5. Record any directions where full tilt becomes walking or partial movement.

## 6. Known Issues and Current Understanding

### Automatic Reconnect

Automatic reconnect has not been proven reliable. Earlier experiments showed
that some controller states do not advertise or reconnect in a simple way after
sleep/disconnect.

Current decision:

- Do not keep destabilizing the firmware for reconnect experiments.
- Preserve the known-good connection path.
- Revisit reconnect later only with better NS2Pro-specific evidence.

### Raw USB Passthrough

Raw passthrough can make the stick range worse. The current understanding is
that a BLE FD2 payload is not necessarily equivalent to the controller's native
wired USB report body.

Current decision:

- Default raw passthrough off.
- Keep it as a diagnostic switch.
- Normal output should use parsed and repacked reports.

### Stick Outer Boundary

Wireless stick range can differ from native USB range. The current firmware uses
center calibration and fixed range expansion, which is stable but not as precise
as a true outer-boundary calibration profile.

Observed problem:

- In some games, pushing the stick fully in certain directions may cause the
  character to walk instead of run.

Current theory:

- Some directions do not reach the host's expected full-scale boundary after
  wireless-to-USB mapping.
- This is a mapping/calibration problem more than a basic FD2 parsing problem.

## 7. Future Calibration Plan

The preferred future solution is a stick calibration flow in the WebHID tuner.

Suggested flow:

1. Ask the user to release both sticks.
2. Record neutral center values.
3. Ask the user to rotate the left stick around the outer edge several times.
4. Ask the user to rotate the right stick around the outer edge several times.
5. Record observed maximum outer boundary data.
6. Save calibration data to flash.
7. Apply the calibration profile when generating USB reports.

Possible implementation levels:

- Level 1: global stick scale factor.
- Level 2: per-stick X/Y scale factors.
- Level 3: per-angle outer-boundary table.

Preferred long-term approach:

- Level 3, because the observed outer boundary may be irregular rather than a
  simple circle or ellipse.

Practical first implementation:

- Start with Level 1 or Level 2 as a safe tuning feature.
- Keep the data format extensible so Level 3 can be added later.

## 8. Release Checklist

Before publishing to GitHub:

- Keep `LICENSE`.
- Keep `NOTICE.md`.
- Keep `LICENSES/`.
- Make sure DS5Dongle and y700-switch2-pro-bridge attribution remains clear.
- Do not claim affiliation with Nintendo, Sony, Raspberry Pi, or other vendors.
- Make sure generated build artifacts are intentional.
- Verify default settings are stable:
  - display off;
  - raw passthrough off;
  - parsed USB output on;
  - settings save/load works.

## 9. Useful Files

- `src/ns2/ns2_ble.cpp` - BLE connection flow.
- `src/ns2/ns2_gatt.cpp` - GATT discovery and controller initialization.
- `src/ns2/ns2_input.cpp` - FD2 input parsing and center handling.
- `src/ns2/ns2_usb.cpp` - Nintendo-style USB report generation, rumble, WebHID
  feature commands.
- `src/ns2/ns2_state.cpp` - persistent runtime settings.
- `src/ns2/ns2_display.cpp` - optional ST7789 display.
- `tools/ns2-webhid-tuner.html` - WebHID configuration and visualization page.

## 10. Non-Goals for the Current Stage

- Multi-controller support.
- Full protocol-perfect NS2Pro USB emulation.
- Audio/headset support.
- Making display output always-on.
- Aggressive automatic reconnect experiments.
- Replacing the current stable parsed-output path with raw passthrough.
