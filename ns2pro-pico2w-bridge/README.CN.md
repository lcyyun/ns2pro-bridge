# NS2Pro Pico2W Bridge

`NS2Pro Pico2W Bridge` 是一个基于 Raspberry Pi Pico 2 W 的 NS2Pro / Switch 2 Pro 风格手柄桥接固件项目。

当前目标是把 NS2Pro 手柄通过蓝牙连接到 Pico 2 W，再由 Pico 2 W 模拟 Nintendo 风格 USB HID 设备输出给主机。

## 当前功能

- NS2Pro BLE 自动连接和手动配对流程。
- Nintendo USB HID 模拟。
- FD2 原始输入透传模式。
- 解析重打包模式，并带摇杆外圈归一化。
- ST7789 240x240 状态屏显示。
- WebHID 调参页面，支持中英文切换。
- 屏幕、Raw USB、解析输入、回报率、震动参数等运行时设置。
- HD 震动转发实验。

## 目录说明

- `src/ns2/`：NS2Pro BLE、输入解析、USB HID、屏幕、配置和状态代码。
- `tools/ns2-webhid-tuner.html`：WebHID 调参页面。
- `tools/serve-ns2-webhid-tuner.js`：本地 WebUI 静态服务器。
- `tools/ds5-bridge-ns2pro.uf2`：当前最新测试固件。
- `docs/NS2PRO_PICO2W_REQUIREMENTS.md`：需求文档和阶段记录。
- `README_DS5Dongle_ORIGINAL.md`：保留的上游 DS5Dongle README。

## 构建

Windows 下使用：

```powershell
powershell.exe -ExecutionPolicy Bypass -File tools\build-windows.ps1
```

NS2Pro 固件需要 `ENABLE_NS2PRO=ON`。当前已生成的 UF2 位于：

```text
tools\ds5-bridge-ns2pro.uf2
```

## WebUI

启动本地 WebUI：

```powershell
node tools\serve-ns2-webhid-tuner.js
```

然后在 Chrome 或 Edge 中打开显示的 localhost 地址，使用 `连接` 或 `使用已授权设备`。

当前设置逻辑：

- `应用`：只对当前上电周期立即生效。
- `保存`：写入 flash，断电后仍然保留。
- `USB 原始透传` 开启：直接透传 FD2 原始输入。
- `USB 原始透传` 关闭：使用解析重打包模式，并启用摇杆外圈归一化。

## 当前测试方法

测试摇杆外圈修正时：

1. 在 WebUI 里关闭 `USB 原始透传`。
2. 点击 `应用`。
3. 确认 `parsed_reports` 增加，`raw_passthrough_reports` 不再增加。
4. 测试摇杆点位分布和游戏内是否能稳定跑步。
5. 确认效果没问题后，再点击 `保存`。

需要对照原始输入时，再打开 `USB 原始透传` 并点击 `应用`。
