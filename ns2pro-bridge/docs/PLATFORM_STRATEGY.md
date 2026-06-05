# Platform Maintenance Strategy

Date: 2026-06-05
Project: `ns2pro-bridge`

## 1. Policy

`ns2pro-bridge` should support multiple hardware targets without requiring all
targets to have identical features or identical release timing.

The project should treat Pico 2 W and ESP32-S3 as separate firmware targets
under one repository:

```text
Pico 2 W  -> stable current target
ESP32-S3  -> experimental/port target
```

The shared project goal is NS2Pro bridging. The per-platform implementation,
feature set, and release cadence may differ.

## 2. Branch Model

Recommended branch roles:

```text
main       Stable project branch. Release-ready code and documentation.
esp32s3    ESP32-S3 port and platform-specific work.
pico2w/*   Short-lived Pico 2 W fixes or features.
esp32s3/*  Short-lived ESP32-S3 fixes or features.
shared/*   Shared parser/report/mapping logic work.
```

Current branch state:

```text
main     Pico 2 W stable release line.
esp32s3  ESP32-S3 planning and port branch.
```

Long term, `main` may contain both platform targets after ESP32-S3 is mature.
That still does not mean the two platforms must expose the same feature set.

## 3. Versioning

Use platform-specific tags when release cadence differs:

```text
pico2w-v0.1.0
pico2w-v0.1.1
esp32s3-v0.1.0
pico2w-v0.1.2
esp32s3-v0.1.1
```

Use combined tags only when both platforms are intentionally released together:

```text
v0.3.0
```

The existing first release is:

```text
v0.1.0 -> Pico 2 W only
```

Future releases should make the target explicit in the release title and asset
names.

## 4. Release Assets

Pico 2 W assets:

```text
ns2pro-bridge-pico2w.uf2
```

Possible ESP32-S3 assets:

```text
ns2pro-bridge-esp32s3.bin
bootloader.bin
partition-table.bin
```

ESP32-S3 release packaging depends on the final ESP-IDF build and flashing
strategy.

## 5. Capability Matrix

Maintain a platform capability table in user-facing docs once ESP32-S3 work
starts.

Initial matrix:

| Feature | Pico 2 W | ESP32-S3 |
| --- | --- | --- |
| BLE controller connection | Stable enough for current testing | Planned |
| FD2 input parsing | Stable | Planned |
| Parsed USB HID output | Stable | Planned |
| Raw USB passthrough | Diagnostic only | Diagnostic only, if implemented |
| WebHID tuner | Stable | Unknown |
| Rumble forwarding | Experimental | Planned |
| Motion parsing | Present, needs validation | Planned |
| ST7789 display | Optional, off by default | Not planned initially |
| Stick outer calibration | Planned | Planned |
| Automatic reconnect | Limited / not solved | Unknown |

The matrix should be updated by evidence from hardware tests, not assumptions.

## 6. Shared Code Policy

Good candidates for shared logic:

- FD2 report parsing.
- Nintendo-style USB report packing.
- Stick center/deadzone/range mapping.
- Future stick outer-boundary calibration data model.

Do not force platform glue into shared code:

- Pico 2 W BLE uses BTstack.
- ESP32-S3 BLE is expected to use ESP-IDF NimBLE.
- Pico 2 W USB uses TinyUSB through Pico SDK.
- ESP32-S3 USB should use the ESP-IDF USB/TinyUSB path if the board supports
  native USB device mode.

Shared code should be extracted only when it reduces real duplication and does
not make either platform harder to debug.

## 7. Documentation Rules

Every platform-specific feature should say which hardware target it applies to.

Avoid wording like:

```text
The firmware supports X.
```

Prefer:

```text
Pico 2 W supports X.
ESP32-S3 does not support X yet.
```

This prevents users from assuming both firmware targets behave the same.

## 8. Local Worktree Recommendation

For Codex usage, the clean local layout should eventually be:

```text
controller/
  ns2pro-bridge/
    main/       worktree for the `main` branch
    esp32s3/    worktree for the `esp32s3` branch
  references/
    DS5Dongle/
    y700-switch2-pro-bridge/
```

Use `git worktree` for this layout instead of copying directories manually.

This keeps two Codex conversations from editing the same working tree while
still sharing one GitHub repository.
