# ns2pro-bridge

`ns2pro-bridge` 是一个基于 Raspberry Pi Pico 2 W 的固件项目。它通过 BLE
连接 NS2Pro 手柄，然后把 Pico 2 W 模拟成 Nintendo 风格 USB HID 手柄输出给主机。

当前目标是先做好单手柄桥接，把已经测试稳定的路径固定下来，而不是一次性追求
完整协议覆盖。

## 当前状态

已经完成并经过测试的部分：

- 单个 NS2Pro 手柄的 BLE 配对和自动连接流程。
- FD2 BLE 输入包解析，包括按键、摇杆、加速度计和陀螺仪。
- Nintendo 风格 USB HID 输出，默认使用“解析后重新打包”的稳定路径。
- HD 震动转发实验。
- WebHID 调参页面，可查看状态、调整参数、测试震动、查看实时输入。
- WebUI 支持中文和英文切换。
- 可选 ST7789 240x240 状态屏。
- WebHID 设置可以保存到 flash，断电后保留。

当前默认行为：

- USB 输出默认使用解析重打包模式。
- USB 原始透传默认关闭。
- ST7789 屏幕默认关闭。
- WebUI 实时动画开关只影响网页显示，不改变 USB 输出模式。

## 重要说明

`USB 原始透传` 只是诊断模式。它会把 BLE FD2 payload 直接塞进 USB report
body，用来对比数据，但这不等价于手柄原生 USB 直连时的报告语义。实际测试中，
关闭原始透传、使用解析重打包模式时，摇杆范围更正确；打开原始透传反而可能出现
外圈到不了边界的问题。

当前摇杆处理已经包含中心校准和固定范围放大。这个方案目前足够稳定，但无线模式下
的摇杆外圈仍然可能和 USB 直连不同。后续计划加入校准流程：提示用户旋转摇杆几圈，
记录实际外圈，再把补偿参数保存到 flash。

## 硬件

必需：

- Raspberry Pi Pico 2 W。
- NS2Pro 手柄。
- 一根 USB 线，用于供电、烧录、USB HID 输出和 WebHID 调参。
- Chrome 或 Edge 浏览器，用于打开 WebHID 调参页。

可选：

- DAPLink 或其他串口调试器，用于查看日志。
- ST7789 240x240 SPI 屏幕，用于设备状态显示。

## 目录说明

- `src/ns2/`：NS2Pro BLE、GATT、输入解析、USB HID、屏幕、配置和状态代码。
- `tools/ns2-webhid-tuner.html`：WebHID 调参页面。
- `tools/serve-ns2-webhid-tuner.js`：本地 WebUI 静态服务器。
- `tools/ds5-bridge-ns2pro.uf2`：当前本地最新 UF2 固件产物。
- `docs/PROJECT_NOTES.md`：当前决策、已知问题和后续计划。
- `NOTICE.md`：上游项目来源、借鉴位置和第三方协议说明。
- `LICENSES/`：保留的上游项目协议文本。

## 构建

Windows 下使用项目自带脚本：

```powershell
powershell.exe -ExecutionPolicy Bypass -File tools\build-windows.ps1
```

NS2Pro 固件使用 `ENABLE_NS2PRO=ON` 构建。当前 UF2 固件位置：

```text
tools\ds5-bridge-ns2pro.uf2
```

## 烧录

让 Pico 2 W 进入 BOOTSEL 模式，然后把 UF2 复制到 `RPI-RP2` 磁盘。

```powershell
Copy-Item -LiteralPath tools\ds5-bridge-ns2pro.uf2 -Destination E:\ -Force
```

其中 `E:\` 需要替换成你电脑上实际出现的 BOOTSEL 盘符。

## WebHID 调参页

启动本地 WebUI：

```powershell
node tools\serve-ns2-webhid-tuner.js
```

然后在 Chrome 或 Edge 中打开显示出来的 localhost 地址，点击 `Connect` 或
`Use Existing`。

设置逻辑：

- `Apply / 应用`：只对当前上电周期立即生效。
- `Save / 保存`：写入 flash，断电后仍然保留。
- `Raw USB (Diag) / 原始透传(诊断)`：正常使用时建议关闭。
- `Live Visuals / 实时可视化`：只控制网页动画和解析显示。
- `Display / 屏幕`：控制可选 ST7789 屏幕，默认关闭。
- `Report Hz / 回报率`：设置目标 USB 回报率。

## 推荐测试流程

1. 保持 `USB 原始透传` 关闭。
2. 点击 `应用`。
3. 确认手柄已连接，并且 parsed reports 持续增加。
4. 测试按键、摇杆、体感和震动。
5. 在游戏或手柄测试页面里测试摇杆外圈和角色跑步状态。
6. 确认设置没有问题后，再点击 `保存`。

## 后续计划

后续比较值得做的内容：

- 摇杆外圈校准：
  - 记录摇杆松手中心点；
  - 提示用户旋转每个摇杆几圈；
  - 记录每个摇杆实际外圈；
  - 保存补偿参数到 flash；
  - USB 重打包时使用校准结果修正外圈。
- 优化屏幕模式：
  - 默认继续关闭屏幕；
  - 打开后降低刷新开销；
  - 显示简洁状态，不再追求高频实时数据。
- 进一步验证体感方向和数值范围。
- 继续微调震动参数。
- 等有更可靠的 NS2Pro 资料后，再重新研究 BLE 自动重连。

## 来源和协议

本项目基于 MIT 协议的 DS5Dongle Pico 固件整理而来，并参考了 Apache-2.0
协议的 y700-switch2-pro-bridge 项目中关于 NS2Pro 的协议分析。

发布到 GitHub 或重新分发固件前，请保留 `NOTICE.md` 和 `LICENSES/`。本仓库
主协议为 MIT，除非具体文件或捆绑的第三方组件另有说明。
