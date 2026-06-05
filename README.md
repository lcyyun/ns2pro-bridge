# ns2pro-bridge

`ns2pro-bridge` is a Raspberry Pi Pico 2 W firmware project that connects an
NS2Pro controller over BLE and exposes it to the host as a Nintendo-style USB
HID controller.

The project currently targets a single controller and prioritizes a stable,
tested bridge path over full protocol completeness.

## Current Status

Working and tested:

- BLE pairing and automatic connection flow for one NS2Pro controller.
- FD2 BLE input parsing for buttons, sticks, accel, and gyro.
- Nintendo-style USB HID output using parsed and repacked input reports.
- Basic HD-rumble forwarding experiments.
- WebHID tuner for runtime settings, status, rumble tests, and live input view.
- English and Chinese WebUI.
- Optional ST7789 240x240 status display support.
- Settings persistence to flash through the WebHID tuner.

Current default behavior:

- USB output uses parsed-report mode.
- Raw USB passthrough is disabled by default.
- ST7789 display output is disabled by default.
- WebUI live visualization can be enabled or disabled without changing USB
  output behavior.

## Important Notes

Raw USB passthrough is a diagnostic mode only. It copies the BLE FD2 payload
into the USB report body for comparison, but the BLE payload is not guaranteed
to match the controller's native wired USB report semantics. In testing, parsed
and repacked USB output gives the correct stick range, while raw passthrough may
fail to reach the expected outer boundary.

The current stick handling uses center calibration and a fixed range expansion.
This is stable enough for normal testing, but wireless stick outer boundaries
can still differ from wired USB behavior. A future calibration flow is planned
to ask the user to rotate each stick and save an outer-boundary compensation
profile to flash.

## Hardware

Required:

- Raspberry Pi Pico 2 W.
- NS2Pro controller.
- USB cable for power, flashing, USB HID output, and WebHID configuration.
- Chrome or Edge for the WebHID tuner.

Optional:

- DAPLink or another serial adapter for debug logs.
- ST7789 240x240 SPI display for on-device status.

## Project Layout

- `src/ns2/` - NS2Pro BLE, GATT, input parsing, USB HID, display, config, and
  status code.
- `tools/ns2-webhid-tuner.html` - browser-based WebHID tuner.
- `tools/serve-ns2-webhid-tuner.js` - local static server for the tuner.
- `tools/ns2pro-bridge-pico2w.uf2` - latest local UF2 firmware artifact.
- `docs/PROJECT_NOTES.md` - current decisions, known issues, and roadmap.
- `docs/ESP32S3_HANDOFF.md` - handoff notes for the experimental ESP32-S3
  branch.
- `NOTICE.md` - upstream attribution and third-party license notes.
- `LICENSES/` - copied upstream license texts for referenced projects.

## Build

On Windows, use the bundled build script:

```powershell
powershell.exe -ExecutionPolicy Bypass -File tools\build-windows.ps1
```

The build script defaults to the NS2Pro Pico 2 W firmware. The current UF2
artifact is kept at:

```text
tools\ns2pro-bridge-pico2w.uf2
```

## Flash

Put the Pico 2 W into BOOTSEL mode, then copy the UF2 file to the mounted
`RPI-RP2` drive.

```powershell
Copy-Item -LiteralPath tools\ns2pro-bridge-pico2w.uf2 -Destination E:\ -Force
```

Replace `E:\` with the actual BOOTSEL drive letter.

## WebHID Tuner

Start the local tuner server:

```powershell
node tools\serve-ns2-webhid-tuner.js
```

Open the displayed localhost URL in Chrome or Edge, then click `Connect` or
`Use Existing`.

Settings behavior:

- `Apply` changes settings for the current boot.
- `Save` writes settings to flash so they survive power loss.
- `Raw USB (Diag)` should normally stay off.
- `Live Visuals` only controls the WebUI parser and animations.
- `Display` controls the optional ST7789 output and is off by default.
- `Report Hz` sets the target USB report rate.

## Recommended Test Flow

1. Keep `Raw USB (Diag)` off.
2. Click `Apply`.
3. Confirm the controller is connected and parsed reports are increasing.
4. Test buttons, sticks, motion, and rumble.
5. Test stick edge behavior in a game or controller test page.
6. Click `Save` only after the settings are confirmed good.

## Roadmap

Planned or likely future work:

- Stick outer-boundary calibration:
  - record neutral center;
  - ask the user to rotate each stick several times;
  - save per-stick outer-boundary compensation to flash;
  - use the calibration profile when repacking USB reports.
- Better display mode:
  - keep display disabled by default;
  - reduce refresh cost when enabled;
  - show compact state instead of high-frequency live data.
- More complete motion validation.
- Rumble tuning refinement.
- Revisit BLE reconnect behavior when better NS2Pro-specific information is
  available.

## Attribution and License

This project is based on the MIT-licensed DS5Dongle Pico firmware and uses
NS2Pro protocol knowledge from the Apache-2.0-licensed
y700-switch2-pro-bridge project. See `NOTICE.md` and `LICENSES/` before
publishing or redistributing builds.

The main repository license is MIT unless a file or bundled third-party
component states otherwise.
