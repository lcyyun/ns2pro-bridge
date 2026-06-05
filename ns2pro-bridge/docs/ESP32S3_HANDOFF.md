# ESP32-S3 Port Handoff

Date: 2026-06-05
Branch: `esp32s3`
Base project: `ns2pro-bridge`

## 1. Purpose

This document is for a new Codex conversation that will explore an ESP32-S3
compatible version of `ns2pro-bridge`.

The current `main` branch is the stable Raspberry Pi Pico 2 W version. The
`esp32s3` branch exists so ESP32-S3 work can happen without disturbing the Pico
2 W firmware, release workflow, or published `v0.1.0` UF2.

## 2. Current Stable State

The stable firmware target is:

```text
NS2Pro controller -> BLE -> Pico 2 W -> Nintendo-style USB HID -> host
```

Stable decisions from the Pico 2 W version:

- Use parsed FD2 input and repack it into USB reports.
- Keep raw FD2 USB passthrough disabled by default.
- Treat raw passthrough as diagnostic only.
- Keep ST7789 display disabled by default.
- Use WebHID as the main tuning interface.
- Do not destabilize the current BLE connection path for reconnect experiments.
- Stick outer-boundary calibration is planned for later, not solved yet.

Release page:

```text
https://github.com/lcyyun/ns2pro-bridge/releases/tag/v0.1.0
```

Current release asset:

```text
ns2pro-bridge-pico2w.uf2
```

## 3. Important Boundaries

Do not rewrite the Pico 2 W implementation while starting the ESP32-S3 port.

Do not mix ESP-IDF/NimBLE code directly into the Pico SDK/BTstack build path.

Do not change the `main` branch for ESP32-S3 experiments. Work on `esp32s3`.

Do not remove attribution files:

- `LICENSE`
- `NOTICE.md`
- `LICENSES/`

Do not claim the ESP32-S3 port is working before it has been built and tested on
hardware.

## 4. Suggested First Step

Start with planning and directory layout, not implementation.

Recommended layout:

```text
firmware/
  pico2w/       optional future home for the current Pico 2 W firmware
  esp32s3/      ESP-IDF implementation, isolated from Pico SDK code
docs/
tools/
```

Practical first step for this branch:

```text
firmware/esp32s3/
  README.md
  CMakeLists.txt
  main/
    CMakeLists.txt
    app_main.cpp
```

The first ESP32-S3 commit should preferably be a scaffold and design note, not a
large partial port.

## 5. Files To Read First

Read these before writing code:

- `README.md`
- `README.CN.md`
- `docs/PROJECT_NOTES.md`
- `NOTICE.md`
- `src/ns2/ns2_input.cpp`
- `src/ns2/ns2_usb.cpp`
- `src/ns2/ns2_gatt.cpp`
- `src/ns2/ns2_ble.cpp`
- `tools/ns2-webhid-tuner.html`

Reference project outside this repo, if available locally:

```text
C:\Users\Lenovo\Documents\controller\y700-switch2-pro-bridge
```

Use it as a protocol and ESP-IDF/NimBLE reference, but do not blindly copy large
blocks without preserving license/notice requirements.

## 6. Technical Questions To Answer Before Coding

The new conversation should answer these first:

1. Which ESP32-S3 board is the target?
2. Does the board expose native USB device pins to the host?
3. Is the USB output target TinyUSB HID under ESP-IDF?
4. Should ESP32-S3 use NimBLE central mode to connect to NS2Pro?
5. Can the WebHID tuner remain compatible with the ESP32-S3 USB HID interface?
6. Should the ESP32-S3 version share report packing logic with Pico 2 W, or keep
   a separate implementation first?
7. How should build artifacts be named?
8. Should release assets eventually include both Pico 2 W and ESP32-S3 builds?

## 7. Recommended Architecture

Keep these layers separate:

```text
BLE transport:
  Pico 2 W: BTstack
  ESP32-S3: ESP-IDF NimBLE

Input parser:
  FD2 buttons/sticks/motion parsing
  Should be kept logically similar across platforms.

USB output:
  Pico 2 W: TinyUSB through Pico SDK
  ESP32-S3: TinyUSB through ESP-IDF, if native USB device is available.

Configuration:
  Pico 2 W: current flash config and WebHID feature reports
  ESP32-S3: NVS or compatible feature-report storage.
```

The best shared logic candidates are:

- FD2 input parsing.
- Nintendo-style USB report packing.
- Stick center/deadzone/range mapping.
- Future stick calibration data model.

Avoid sharing platform glue too early.

## 8. Known Risks

- ESP32-S3 native USB support depends on board wiring and ESP-IDF configuration.
- The current WebHID tuner assumes the existing HID feature report protocol.
- Raw FD2 passthrough is not a proven correct USB output path.
- Automatic reconnect is not solved on Pico 2 W and should not be treated as
  solved for ESP32-S3.
- Rumble forwarding and motion orientation still need real hardware validation.

## 9. Suggested Prompt For The New Conversation

Copy this into the new conversation:

```text
We are working on the `esp32s3` branch of:

https://github.com/lcyyun/ns2pro-bridge

Do not modify `main`. Do not break the existing Pico 2 W firmware.

Please first read:

- README.md
- README.CN.md
- docs/PROJECT_NOTES.md
- docs/ESP32S3_HANDOFF.md
- NOTICE.md
- src/ns2/ns2_input.cpp
- src/ns2/ns2_usb.cpp
- src/ns2/ns2_gatt.cpp
- src/ns2/ns2_ble.cpp

Goal: plan an ESP32-S3 compatible version of the NS2Pro bridge.

Important constraints:

- Keep ESP32-S3 code isolated from Pico SDK/BTstack code.
- Prefer an ESP-IDF project under firmware/esp32s3/.
- Use y700-switch2-pro-bridge only as a reference with proper attribution.
- Do not implement a large port immediately.
- First produce a directory plan, build plan, and risk list.
- After the plan is approved, scaffold the ESP32-S3 project.
```

## 10. Definition Of Done For The First ESP32-S3 Step

The first useful ESP32-S3 step is done when:

- The branch has an isolated ESP32-S3 project scaffold.
- Pico 2 W build and release files are not broken.
- README or docs explain that ESP32-S3 is experimental.
- No untested ESP32-S3 code is described as working.
- The next hardware test steps are clear.
