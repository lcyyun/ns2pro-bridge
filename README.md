# NS2Pro Bridge

NS2Pro Bridge is a Raspberry Pi Pico 2 W firmware project for bridging an
NS2Pro / Switch 2 Pro style Bluetooth controller to a Nintendo-style USB HID
device.

The current tested path focuses on Pico 2 W:

- BLE auto connection and manual pairing flow for the NS2Pro controller.
- Nintendo USB HID presentation with raw FD2 passthrough and parsed-report mode.
- ST7789 240x240 status display support.
- WebHID tuning page with English / Chinese UI.
- Runtime controls for display, raw USB passthrough, parsed WebUI input,
  report rate, and rumble tuning.
- HD rumble forwarding experiments.

## Project Layout

- `src/ns2/` - NS2Pro BLE, input parsing, USB HID, display, config, and status code.
- `tools/ns2-webhid-tuner.html` - browser-based WebHID tuner.
- `tools/serve-ns2-webhid-tuner.js` - local static server for the tuner.
- `tools/ds5-bridge-ns2pro.uf2` - latest tested UF2 firmware artifact.
- `docs/NS2PRO_PICO2W_REQUIREMENTS.md` - original requirements and task notes.
- `NOTICE.md` - upstream attribution and third-party license notes.
- `LICENSES/` - copied upstream licenses for referenced projects.

## Build

On Windows, use the bundled builder:

```powershell
powershell.exe -ExecutionPolicy Bypass -File tools\build-windows.ps1
```

For the NS2Pro firmware, build with `ENABLE_NS2PRO=ON`. The current generated
UF2 is also kept at:

```text
tools\ds5-bridge-ns2pro.uf2
```

## Web Tuner

Start the local tuner server:

```powershell
node tools\serve-ns2-webhid-tuner.js
```

Open the displayed localhost URL in Chrome or Edge, then use `Connect` or
`Use Existing`.

Current behavior:

- `Apply` changes settings immediately for the current boot.
- `Save` writes settings to flash so they survive power loss.
- `Raw USB` on means raw FD2 passthrough.
- `Raw USB` off means parsed report mode with joystick normalization enabled.

## Current Test Notes

The current preferred joystick test is:

1. Turn `Raw USB` off in the WebUI.
2. Click `Apply`.
3. Confirm `parsed_reports` increases and `raw_passthrough_reports` stops increasing.
4. Test stick edge distribution and in-game full-run behavior.
5. Click `Save` only after the setting is known good.

If raw passthrough is needed for comparison, turn `Raw USB` back on and click
`Apply`.

## Attribution and License

This project is based on the MIT-licensed DS5Dongle Pico firmware and uses
NS2Pro / Switch 2 Pro protocol knowledge from the Apache-2.0-licensed
y700-switch2-pro-bridge project. See `NOTICE.md` and `LICENSES/` before
publishing or redistributing builds.

The main repository license is MIT unless a file or bundled third-party
component states otherwise.
