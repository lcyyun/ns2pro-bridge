# NS2Pro Bridge

`NS2Pro Bridge` 是一个基于 Raspberry Pi Pico 2 W 的 NS2Pro / Switch 2 Pro 风格手柄桥接固件项目。
当前目标是把 NS2Pro 手柄通过蓝牙连接到 Pico 2 W，再由 Pico 2 W 模拟 Nintendo 风格 USB HID 设备输出给主机。

## 当前功能

- NS2Pro BLE 自动连接和手动配对流程。
- Nintendo USB HID 模拟。
- 默认使用解析输出模式，并带摇杆外圈归一化。
- FD2 原始输入透传保留为实验性诊断模式。
- ST7789 240x240 状态屏可选，默认关闭。
- WebHID 调参页面，支持中英文切换。
- 屏幕、原始透传诊断、网页实时可视化、回报率、震动参数等运行时设置。
- HD 震动转发实验。

## 目录说明

- `src/ns2/`：NS2Pro BLE、输入解析、USB HID、屏幕、配置和状态代码。
- `tools/ns2-webhid-tuner.html`：WebHID 调参页面。
- `tools/serve-ns2-webhid-tuner.js`：本地 WebUI 静态服务器。
- `tools/ds5-bridge-ns2pro.uf2`：当前最新测试固件。
- `docs/NS2PRO_PICO2W_REQUIREMENTS.md`：需求文档和阶段记录。
- `NOTICE.md`：上游项目来源、借鉴位置和第三方协议说明。
- `LICENSES/`：复制保留的上游项目协议文本。

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
- `原始透传(诊断)` 关闭：使用当前测试正确的解析输出模式。
- `原始透传(诊断)` 开启：直接透传 FD2 负载，仅用于对照诊断；它不一定等同于手柄 USB 直插，摇杆边界可能不正确。
- `实时可视化` 只控制网页里的摇杆和体感动画，不改变 USB 输出模式。

## 当前测试方法

测试摇杆外圈修正时：

1. 在 WebUI 里保持 `原始透传(诊断)` 关闭。
2. 点击 `应用`。
3. 确认 `parsed_reports` 增加，`raw_passthrough_reports` 不再增加。
4. 测试摇杆点位分布和游戏内是否能稳定跑步。
5. 确认效果没问题后，再点击 `保存`。

需要对照原始输入时，再打开 `原始透传(诊断)` 并点击 `应用`；正常使用时再关回去。

## 来源和协议

本项目基于 MIT 协议的 DS5Dongle Pico 固件代码整理而来，并参考了 Apache-2.0 协议的
y700-switch2-pro-bridge 项目中关于 NS2Pro / Switch 2 Pro 的协议分析。

发布到 GitHub 或重新分发固件前，请保留 `NOTICE.md` 和 `LICENSES/`。本仓库主协议为 MIT，
除非具体文件或捆绑的第三方组件另有说明。
