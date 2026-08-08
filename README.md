# esphome-for-HLK-LD2402

HLK-LD2402 毫米波雷达的 ESPHome 自定义组件。基础代码来自
[bbs.hassbian.com 帖子](https://bbs.hassbian.com/thread-29879-1-1.html)，本仓库在此基础上做了
适配最新 ESPHome 与全面非阻塞化改造。

> 此组件可同时在 esp8266 及 esp32 上运行（仓库含 `hlk_ld2402_esp8266.yaml` 示例配置）。

## 硬件连接（ESP8266 / ESP-12E）

| 雷达 | ESP8266 |
| ---- | ------- |
| TX   | GPIO3 (RX0) |
| RX   | GPIO1 (TX0) |
| OUT  | GPIO12（可选，存在检测 GPIO 输出） |
| VCC  | 5V（板载 3.3V 稳压） |
| GND  | GND |

## 使用

```bash
# 1. 复制示例配置并填写 secrets.yaml（WiFi 密码、API 密钥等）
cp hlk_ld2402_esp8266.yaml my_ld2402.yaml
# 2. 编辑 my_ld2402.yaml 中的 external_components（本地路径或 git 源）
# 3. 编译烧录
esphome run my_ld2402.yaml
```

配置要点：

- `external_components` 默认使用本地 `components/` 目录（离线可编译）；如需跟随上游，
  换回 git 源即可：
  ```yaml
  external_components:
    - source:
        type: git
        url: https://github.com/sisheng36/esphome-for-HLK-LD2402
  ```
- 所有功能均通过实体/服务暴露：校准、自动增益、保存配置、工程模式、
  单门阈值设置/批量读取、恢复出厂等（见 YAML 中 button/number/select 部分）。

## 与最新 ESPHome 的兼容性（要求 ≥ 2026.7.0）

- **API**：`encryption: key` 必须使用 base64 密钥；`api: password` 已移除（2026.1.0）
- **OTA**：2026.1.0 起强制 SHA256。若设备固件版本低于 2025.10，请**分步升级**
  （先刷 2025.12.x，再刷当前版本），否则无法直接 OTA
- **web_server**：使用 v2 写法（`web_server: version: 2`）；v1（`port: 80`）已弃用，
  2027.1.0 移除
- **ESP8266 内存**：2026.1–2026.4 的内存优化（Serial 排除、StringRef、回调精简等）
  自动生效，典型配置可用堆从不足 10KB 提升到 30KB 以上。升级后建议观察
  `logger` 中 heap 余量，运行 24h 无重启/掉线为正常
- **power_save_mode**：2026.3.1 修复了 LIGHT/HIGH 映射颠倒的问题，若配置过省电模式
  请复查取值（默认 NONE 不受影响）
- 组件不直接使用 Arduino `Serial`，无需 `enable_serial`；所有 UART 交互均通过
  ESPHome `uart` 组件进行

## 已知限制

- 文本行（`distance:`/`OFF`）与二进制帧（工程模式数据帧）共用 UART 接收状态机，
  异常文本不会破坏帧同步
- 传感器数量较多（约 70 个实体），1MB flash 的设备（如 D1 Mini Lite、ESP-01）可能
  编译超限，建议使用 4MB 模块（ESP-12E 等）

## License

[GPL-3.0](LICENSE) © 2025-2026 [sisheng36](https://github.com/sisheng36)
（基础代码来自 bbs.hassbian.com 社区分享，遵循原分享许可）
