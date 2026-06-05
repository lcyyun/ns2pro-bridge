# Notices and Attribution

NS2Pro Bridge is a Raspberry Pi Pico 2 W firmware project derived from and
inspired by several open-source projects.

## Main Code Base

This repository was created from the `DS5Dongle` Pico firmware code base:

- Project: DS5Dongle
- Upstream: https://github.com/awalol/DS5Dongle
- Local reference revision: `8760ee3 fix: ci artifact path`
- License: MIT License
- License copy: `LICENSES/DS5Dongle-MIT.txt`

The Pico SDK integration, TinyUSB/BTstack firmware structure, build scripts,
UF2 workflow, DualSense-oriented fallback firmware files, and much of the
repository layout come from this code base.

## NS2Pro / Switch 2 Pro Protocol Reference

The NS2Pro / Switch 2 Pro controller protocol work in this repository uses
`y700-switch2-pro-bridge` as an important reference:

- Project: y700-switch2-pro-bridge
- Upstream: https://github.com/LeonChrome/y700-switch2-pro-bridge
- Local reference revision: `3697227 Make README bilingual`
- License: Apache License 2.0
- License copy: `LICENSES/y700-switch2-pro-bridge-Apache-2.0.txt`

The Pico implementation is not a direct ESP-IDF port, but it borrows protocol
knowledge and design choices from that project, especially:

- BLE service/characteristic UUIDs and controller discovery heuristics.
- NS2Pro initialization command sequence.
- FD2 input report layout, including stick packing and motion offset notes.
- Nintendo-style USB HID report identity and report ID usage.
- HID OUT to BLE rumble forwarding strategy.
- Report-rate/status concepts used by the local WebHID tuner.

Source files with protocol-derived implementation notes include:

- `src/ns2/ns2_gatt.cpp`
- `src/ns2/ns2_input.cpp`
- `src/ns2/ns2_usb.cpp`
- `src/ns2/ns2_usb_descriptors.cpp`
- `tools/ns2-webhid-tuner.html`

## Bundled Third-Party Components

The repository also contains third-party components inherited from DS5Dongle.
Their license files remain in their original directories:

- Raspberry Pi Pico SDK, TinyUSB, and BTstack are used through the Pico SDK
  build environment.
- Opus codec sources are under `lib/opus/` with their upstream license files.
- Cockos WDL sources are under `lib/WDL/` with their upstream license files.
- Additional small third-party components inside those libraries keep their
  own license files next to the source.

## Trademarks

This project is not affiliated with, endorsed by, or sponsored by Nintendo,
Sony, Valve, Raspberry Pi, or any other hardware/software vendor. Nintendo
Switch, Switch Pro Controller, DualSense, Steam, Raspberry Pi, and related
names are trademarks of their respective owners.
