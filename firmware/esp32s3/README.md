# ns2pro-bridge ESP32-S3 scaffold

Status: experimental hardware-tested ESP32-S3 port. It has been built,
flashed, boot-checked, and tested far enough to scan, connect to an NS2Pro
controller, discover GATT, initialize the controller, subscribe to FD2 input,
forward live input over USB HID, and exercise the WebHID-compatible status and
rumble command paths.

This directory is the isolated ESP-IDF home for the ESP32-S3 port. It is kept
outside the Pico SDK / BTstack build path so ESP32-S3 work does not disturb the
stable Pico 2 W firmware in the repository root.

The current code initializes an ESP-IDF TinyUSB HID path compatible with the
Pico 2 W WebHID tuner framing, starts a NimBLE central scanner, attempts to
connect likely NS2Pro / Nintendo controller advertisements, discovers GATT
services and characteristics, subscribes to ACK/input notifications, sends the
known NS2Pro initialization command sequence, parses FD2/legacy input reports,
packs live input into Nintendo-style USB reports, and translates compatible HID
OUT rumble reports into Pro2 HD BLE rumble packets.

Those paths compile and have been validated against one live controller setup.
Settings and the last live controller target are persisted to NVS. The port is
still experimental: BLE security edge cases, rumble feel, host compatibility,
and long-run reconnect behavior need more hardware time before treating it as
as stable as the Pico 2 W firmware.

## Intended architecture

```text
NS2Pro controller -> BLE -> ESP32-S3 -> USB HID -> host
```

Planned platform choices:

- ESP-IDF app under `firmware/esp32s3/`.
- Default board profile: `n16r8` for ESP32-S3-WROOM-1-N16R8 / DevKitC-style
  hardware with 16 MB Quad SPI flash and 8 MB Octal SPI PSRAM.
- BLE Central through ESP-IDF NimBLE, pending hardware and SDK validation.
- USB HID through ESP-IDF TinyUSB, only for boards exposing native USB device
  pins to the host connector.
- Runtime settings through NVS once the WebHID compatibility layer is wired to
  persistent ESP32-S3 settings.

Logic that may become shared later:

- FD2 input parsing.
- Nintendo-style USB report packing.
- Stick center/deadzone/range mapping.
- Future stick outer-boundary calibration data model.

Keep platform glue separate:

- Pico 2 W BLE remains BTstack.
- ESP32-S3 BLE should use ESP-IDF NimBLE.
- Pico 2 W USB remains TinyUSB through Pico SDK.
- ESP32-S3 USB should use the ESP-IDF TinyUSB device path.

## Build

From this directory, with ESP-IDF installed and activated:

```powershell
idf.py build
```

The project CMake defaults to:

```text
IDF_TARGET=esp32s3
NS2_ESP32S3_BOARD=n16r8
```

The default N16R8 settings are split into small config fragments:

```text
sdkconfig.defaults
boards/n16r8/sdkconfig.defaults
```

To build another board profile later:

```powershell
idf.py -DNS2_ESP32S3_BOARD=<profile> build
```

If `sdkconfig` already exists, remove it or run `idf.py fullclean` before
switching profiles.

On this Windows development machine, the scaffold has been compile-tested with
ESP-IDF v5.3.3 after setting the ESP-IDF environment manually:

```powershell
$env:IDF_PATH="C:\Espressif\frameworks\esp-idf-v5.3.3"
$env:IDF_TOOLS_PATH="$env:USERPROFILE\.espressif"
$env:IDF_PYTHON_ENV_PATH="$env:USERPROFILE\.espressif\python_env\idf5.3_py3.12_env"
$env:ESP_ROM_ELF_DIR="$env:USERPROFILE\.espressif\tools\esp-rom-elfs\20240305"
```

The normal `export.ps1` path should be preferred once the local ESP-IDF
installation can export all tools cleanly.

This port has been flashed and live-tested on 2026-06-06 and 2026-06-07. Tests
confirmed TinyUSB startup, the WebHID-compatible HID identity, NimBLE central
startup, 16 MB flash, 8 MB PSRAM, NS2Pro advertisement matching, GATT discovery,
controller initialization, FD2 notification parsing, USB input forwarding,
disconnect-to-rescan reconnect, and NVS-backed runtime settings.

## Hardware questions to answer

- Exact target ESP32-S3 development board model.
- Whether the board exposes native USB device D+/D- pins to the host-facing USB
  connector.
- Whether USB HID should use ESP-IDF TinyUSB for that board and ESP-IDF version.
- Whether BLE Central should use NimBLE and what security/bonding behavior the
  controller requires on ESP32-S3.
- Whether `tools/ns2-webhid-tuner.html` can be reused by preserving VID/PID,
  report IDs, and feature-report command framing.

## Next implementation steps

1. Confirm board model, native USB wiring, and ESP-IDF version.
2. Build and flash this scaffold to confirm the toolchain and serial logging.
3. Flash this TinyUSB HID / NimBLE scan smoke test and capture boot logs.
4. Confirm the host sees the HID interface and the WebHID tuner can read the
   experimental status feature report.
5. Confirm scan logs identify the NS2Pro controller in pairing mode.
6. Test controller pairing and capture connection / GATT discovery logs.
7. Keep validating FD2 input parsing against live WebUI and host USB behavior.
8. Validate HID OUT to BLE rumble feel across more hosts and games.
9. Harden BLE security/bond recovery and long-run reconnect behavior.

## Attribution

Protocol details are informed by the same references listed in the repository
`NOTICE.md`, especially the Apache-2.0 licensed `y700-switch2-pro-bridge`.
Preserve `NOTICE.md` and `LICENSES/` when publishing or redistributing builds.
