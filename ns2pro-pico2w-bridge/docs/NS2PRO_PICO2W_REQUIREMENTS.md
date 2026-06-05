# Pico 2 W NS2 Pro Auto-Connect Requirements

Date: 2026-06-03
Status: Draft
Target project base: DS5Dongle

## 1. Background

The current DS5Dongle project already provides a clean Pico 2 W firmware base with TinyUSB, BTstack Classic, UF2 flashing, configuration over HID feature reports, and a mature reconnect-oriented receiver flow for DualSense controllers.

The y700-switch2-pro-bridge project has a working ESP32-S3 implementation for NS2 Pro / Switch 2 Pro controller bridging, but its BLE connection logic is ESP-IDF / NimBLE specific and cannot be copied directly to Pico 2 W. The useful pieces are the protocol knowledge, BLE UUIDs, initialization sequence, report parsing, and future USB report mapping.

This document defines the first Pico 2 W milestone only:

Pico 2 W automatically discovers, connects, reconnects, and monitors one NS2 Pro controller over BLE.

## 2. MVP Scope

### In Scope

- Pico 2 W firmware boots and initializes BTstack BLE Central mode.
- Firmware scans for likely NS2 Pro / Switch 2 Pro BLE advertisements.
- Firmware automatically connects to a matching controller.
- Firmware persists the last successful controller address.
- Firmware reconnects to the saved controller after reboot.
- Firmware retries connection after failure or disconnect.
- Firmware subscribes to the controller input notify characteristic.
- Firmware confirms that live input notifications are being received.
- Firmware exposes enough status for debugging through USB serial or HID feature command.
- Firmware has LED status patterns for idle, scanning, connecting, connected, and error.

### Out of Scope for MVP

- DS5 and NS2 Pro combined firmware.
- Simultaneous multi-controller support.
- Full USB gamepad output to Windows / Steam.
- Nintendo USB identity emulation.
- Rumble forwarding.
- Gyro mapping validation.
- Audio, voice, microphone, or headset features.
- Web configuration UI.
- Windows Manager integration.

## 3. Success Criteria

The MVP is considered successful when all of the following are true:

1. With the NS2 Pro in pairing mode, Pico 2 W finds it without manual address entry.
2. Pico 2 W connects to the controller and stores its BLE address.
3. After power cycling Pico 2 W, it reconnects to the same controller when the controller is awake or advertising.
4. If the controller disconnects, Pico 2 W returns to reconnect mode automatically.
5. The firmware reports `connected`, `live_notify=true`, and an increasing notify counter.
6. Manual replug is not required for normal reconnection.
7. The BLE connection remains stable for at least 30 minutes.
8. Reconnection after controller sleep/wake succeeds within a reasonable time, target under 15 seconds once the controller advertises.

## 4. Hardware Requirements

### Required

- Raspberry Pi Pico 2 W.
- USB cable for power, flashing, and debug/status output.
- NS2 Pro / Switch 2 Pro controller.
- Windows PC for flashing and log observation.

### Not Required for MVP

- ESP32-S3.
- PSRAM.
- External Bluetooth dongle.
- Second USB port.
- Battery or portable enclosure.

## 5. Firmware Architecture

The MVP should be implemented as a new NS2 Pro profile inside the DS5Dongle codebase, not as a direct port of ESP32-S3 firmware.

Recommended modules:

```text
src/ns2/ns2_ble.h
src/ns2/ns2_ble.cpp
src/ns2/ns2_gatt.h
src/ns2/ns2_gatt.cpp
src/ns2/ns2_state.h
src/ns2/ns2_state.cpp
src/ns2/ns2_status.h
src/ns2/ns2_status.cpp
```

Existing DS5 modules should remain untouched except for build wiring and shared status/LED hooks.

## 6. BLE Requirements

### Scan Behavior

- Start BLE active scan automatically after boot.
- Use a scan interval and window suitable for responsive discovery without excessive load.
- Recognize likely NS2 Pro candidates by one or more of:
  - advertised name
  - appearance
  - service UUIDs
  - manufacturer data
  - address seen during prior successful connection
- Keep a small candidate cache with address, address type, RSSI, name, and match reason.

### Connect Behavior

- If a saved target exists, attempt direct reconnect first.
- If direct reconnect fails, fall back to scanning for the saved target.
- If no saved target exists, scan and connect to the first strong NS2 Pro candidate.
- On successful connection, persist address and address type.
- Do not require the user to type or copy a BLE address.

### Retry Behavior

- Retry must be continuous while auto-connect is enabled.
- A failed connection attempt must not permanently stop the reconnect loop.
- A disconnect event must return the firmware to reconnect mode.
- Use backoff to avoid tight retry loops.

Recommended retry schedule:

```text
0-30 seconds: retry every 2 seconds
30-120 seconds: retry every 5 seconds
after 120 seconds: retry every 10 seconds
```

### GATT Behavior

- Discover required services and characteristics after connection.
- Discover CCCD descriptors where needed.
- Subscribe to the known input notify characteristic.
- Run the NS2 Pro initialization sequence required to start live reports.
- Track whether live notify packets are actually received.
- Treat `connected but no notify` as a partial failure and expose it in status.

## 7. State Machine

The firmware should use an explicit state machine.

```text
BOOT
  -> BLE_INIT
  -> IDLE
  -> SCANNING
  -> CONNECTING
  -> DISCOVERING
  -> SUBSCRIBING
  -> INITIALIZING_CONTROLLER
  -> CONNECTED_LIVE
  -> DISCONNECTED
  -> SCANNING
```

Failure transitions:

```text
CONNECTING failed -> BACKOFF -> SCANNING
DISCOVERING failed -> DISCONNECT -> BACKOFF -> SCANNING
SUBSCRIBING failed -> DISCONNECT -> BACKOFF -> SCANNING
INITIALIZING_CONTROLLER failed -> DISCONNECT -> BACKOFF -> SCANNING
CONNECTED_LIVE disconnect -> BACKOFF -> SCANNING
CONNECTED but notify timeout -> DISCONNECT -> BACKOFF -> SCANNING
```

## 8. Persistent Configuration

The MVP needs a small persistent config block.

Required fields:

```text
magic
version
crc32
auto_connect_enabled
saved_addr
saved_addr_type
last_success_unix_or_boot_counter
scan_policy
```

Default values:

```text
auto_connect_enabled = true
saved_addr = empty
scan_policy = saved-first-then-any-candidate
```

Config writes must be verified after flash programming.

## 9. Status and Debug Requirements

The firmware must provide a machine-readable status response.

Minimum fields:

```json
{
  "ok": true,
  "profile": "ns2pro",
  "ble_state": "connected_live",
  "auto_connect": "on",
  "saved_target": "aa:bb:cc:dd:ee:ff/0",
  "candidate_count": 1,
  "connected_addr": "aa:bb:cc:dd:ee:ff/0",
  "rssi": -55,
  "notify_count": 12345,
  "notify_hz": 123,
  "last_notify_age_ms": 4,
  "connect_attempts": 3,
  "disconnect_count": 1,
  "last_error": ""
}
```

Recommended commands:

```text
status
ns2 scan
ns2 reconnect
ns2 disconnect
ns2 forget
ns2 auto on
ns2 auto off
ns2 candidates
```

For MVP, these commands may be exposed through USB serial. HID feature command support is optional.

## 10. LED Requirements

Use Pico 2 W onboard LED for basic feedback.

```text
off                    booting or disabled
slow blink 1 Hz         scanning
fast blink 4 Hz         connecting/discovering/subscribing
solid on                connected and live notify active
double blink repeating  connected but no live notify
triple blink repeating  error/backoff
```

## 11. Performance Requirements

- BLE notify parser must avoid heap allocation in the hot path.
- Input notify timestamp must be updated on every valid packet.
- Status counters must be safe to read from the main loop.
- BLE reconnect loop must not block TinyUSB tasks.
- Firmware should remain responsive to status commands during scanning and reconnecting.

Target metrics:

```text
initial discovery: under 30 seconds when controller is in pairing mode
saved reconnect: under 15 seconds after controller advertises
notify rate: record measured rate, expected around 100-130 Hz depending on controller behavior
stable run: 30 minutes minimum without unintended disconnect
```

## 12. Build Requirements

The build should add a new variant:

```text
standard DS5 firmware: existing behavior
ns2pro-connect firmware: NS2 Pro BLE auto-connect MVP
```

Suggested CMake option:

```text
-DENABLE_NS2PRO=ON
```

The NS2 Pro variant must link BTstack BLE support. The existing DS5 firmware currently uses BTstack Classic. The first MVP should not try to enable DS5 and NS2 Pro at the same time.

## 13. Acceptance Test Plan

### Test 1: First Pairing

1. Flash NS2 Pro Pico 2 W firmware.
2. Put NS2 Pro controller into pairing mode.
3. Power Pico 2 W.
4. Confirm LED enters scanning state.
5. Confirm Pico finds and connects.
6. Confirm status shows `connected_live`.
7. Confirm notify counter increases.

Pass condition:

```text
connected_live=true
notify_count increases for 60 seconds
saved_target is non-empty
```

### Test 2: Power Cycle Reconnect

1. Keep controller available.
2. Unplug Pico 2 W.
3. Plug Pico 2 W again.
4. Do not enter manual address.
5. Confirm automatic reconnect.

Pass condition:

```text
Pico reconnects to saved controller without manual command.
```

### Test 3: Controller Sleep/Wake

1. Connect successfully.
2. Let controller sleep or manually power it off.
3. Confirm firmware enters reconnect state.
4. Wake controller.
5. Confirm automatic reconnect.

Pass condition:

```text
No Pico replug required.
Reconnect succeeds after controller advertises.
```

### Test 4: No Controller Present

1. Power Pico 2 W with controller off.
2. Confirm firmware keeps scanning/retrying.
3. Confirm status commands still work.

Pass condition:

```text
No crash.
No watchdog reset.
Status remains readable.
```

### Test 5: Long Run

1. Connect NS2 Pro.
2. Leave connected for 30 minutes.
3. Monitor notify count and disconnect count.

Pass condition:

```text
No unintended disconnect.
last_notify_age_ms remains low during active controller use.
```

## 14. Risks

- Pico 2 W BTstack BLE Central support may require careful SDK configuration.
- DS5Dongle currently uses Classic-only BTstack wiring, so BLE build wiring must be added.
- NS2 Pro BLE initialization sequence may differ by controller firmware version.
- Controller may not advertise continuously after sleep.
- BLE notify parsing must be ported from ESP32 NimBLE code to BTstack callbacks.
- Future USB Nintendo identity may require descriptor switching and host re-enumeration.

## 15. Future Milestones

After MVP passes:

1. Add Nintendo USB HID output on Pico 2 W.
2. Map NS2 Pro input reports to Nintendo/Switch Pro USB reports.
3. Validate Steam recognition.
4. Add rumble forwarding.
5. Add gyro passthrough.
6. Add DS5 / NS2 Pro profile selection.
7. Add auto-detect mode with USB descriptor selected after controller type is known.
8. Add web or HID feature configuration UI.

## 16. Recommended Implementation Order

1. Add `ENABLE_NS2PRO` build option.
2. Add BLE stack wiring for Pico 2 W.
3. Add NS2 state machine and LED states.
4. Implement scan and candidate detection.
5. Implement connect and persistent target save.
6. Implement disconnect/retry/backoff.
7. Implement GATT discovery.
8. Implement notify subscription.
9. Implement NS2 initialization writes.
10. Implement status command.
11. Run acceptance tests.

## 17. MVP Definition

The MVP does not need to make Windows see a gamepad.

The MVP only needs to prove this:

```text
Pico 2 W can automatically find, connect, reconnect, and receive live packets from one NS2 Pro controller.
```

Once this is stable, USB gamepad output becomes the next milestone.
