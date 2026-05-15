# HomeNode

`HomeNode` 是智能家居系统的下位机节点工程（STM32F103C8T6），通过 `ESP-01S` 接入同一局域网，并与上位机 `HomePanel_test` 使用自定义轻量协议进行双向通信。

---

## 1. 当前进度（与代码一致）

当前已实现并持续联调的能力：

- 三路继电器独立控制（USB1/USB2/USB3）。
- 传感数据与状态上报（温度、湿度、模式、风速、输出位）。
- 接收上位机 `CONTROL` 指令并立即执行（继电器 + 风扇状态）。
- 协议 `ACK/ERR` 回应链路。
- ESP 串口 DMA 循环接收，降低了丢字节和串口堵塞问题。
- WiFi/TCP 状态机重连与节奏控制（含失败退避、重开连接）。

---

## 2. 目录说明

- `App/Src/app_node.c`
  - 节点主状态机、AT 流程、协议收发、控制执行逻辑。
- `App/Src/app_home_protocol.c`
  - 协议编解码与 CRC。
- `App/Src/bsp_esp01s.c`
  - ESP 串口发送/接收封装（含 DMA RX 环形读）。
- `App/Src/bsp_relay.c` / `bsp_motor.c` / `bsp_dht11.c`
  - 外设驱动。
- `CMakeLists.txt`
  - 工程构建入口，包含“编译期节点角色选择”。

---

## 3. 编译时选择节点（厨房 / 起居室 / 卧室）

本工程已支持 **编译期节点选择**，不需要再手动改源码中的 `APP_NODE_ID`。

### 3.1 可选角色

- `kitchen` -> 节点 ID = `1`
- `living` -> 节点 ID = `2`
- `bedroom` -> 节点 ID = `3`

### 3.2 方式 A：直接使用预设（推荐）

工程已提供以下 preset：

- `DebugKitchen`
- `DebugLiving`
- `DebugBedroom`

命令示例（PowerShell）：

```powershell
cmake --preset DebugKitchen
cmake --build --preset DebugKitchen
```

```powershell
cmake --preset DebugLiving
cmake --build --preset DebugLiving
```

```powershell
cmake --preset DebugBedroom
cmake --build --preset DebugBedroom
```

### 3.3 方式 B：自定义变量

也可以直接指定缓存变量：

```powershell
cmake -S . -B build/Debug -G Ninja -DHOME_NODE_ROLE=kitchen -DCMAKE_BUILD_TYPE=Debug
cmake --build build/Debug
```

`HOME_NODE_ROLE` 仅支持：`kitchen|living|bedroom`，其他值会在配置阶段直接报错。

---

## 4. 网络与上传参数配置

当前参数在 `App/Src/app_node.c` 顶部定义：

- `APP_WIFI_SSID`
- `APP_WIFI_PASSWORD`
- `APP_UPLOAD_HOST`
- `APP_UPLOAD_PORT`

说明：

- `APP_UPLOAD_HOST` 应配置为上位机 IP。
- 端口与上位机保持一致（当前上位机 TCP server 默认 `5000`）。

---

## 5. 协议摘要

帧格式：

```text
SOF0 SOF1 VER NODE CMD SEQ_L SEQ_H LEN_L LEN_H PAYLOAD CRC_L CRC_H
```

- `SOF0/SOF1` = `0xAA 0x55`
- `VER` = `0x01`
- `CRC16` 范围：从 `VER` 开始到 `PAYLOAD` 结束（不含 SOF 与 CRC 本身）

当前关键命令：

- `TELEMETRY (0x02)`：下位机上传
- `CONTROL (0x03)`：上位机下发
- `ACK (0x06)` / `ERR (0x07)`：双向响应

主要 payload：

- `TELEMETRY`（8 字节）
  - `[0..1]` `temp_x10`（int16 little-endian）
  - `[2..3]` `hum_x10`（uint16 little-endian）
  - `[4]` `mode`
  - `[5]` `fan`
  - `[6]` `online`
  - `[7]` `usb_flags`
- `CONTROL`（5 字节）
  - `[0..1]` `target_temp_x10`
  - `[2]` `mode`
  - `[3]` `fan`
  - `[4]` `usb_flags`

---

## 6. 串口与稳定性策略（当前版本）

当前与稳定性相关的关键策略：

- ESP RX 使用 DMA 循环缓冲读取，减少 `WIFI CONNECTED` 等关键字符串被截断概率。
- `+IPD` 字节流按状态机解析后再喂给协议解析器，避免串扰。
- 控制回包优先，状态上报延迟发送，降低“控制与上报打架”。
- TCP 失败优先尝试恢复连接，不立即 WiFi 全重连。

---

## 7. 快速联调建议

建议顺序：

1. 下位机单独上电，串口确认 WiFi + TCP 能建立。
2. 上位机再入网并启动服务。
3. 先验证上行（TELEMETRY），再测下发（CONTROL）。
4. 压测时避免无节制高频连点，逐步提高频率观察串口日志。

若出现频繁断连，优先检查：

- 热点/AP 是否稳定。
- 上下位机 IP/端口是否一致。
- 是否误把下位机编成了错误节点 ID。

---

## 8. 后续计划（建议）

- 将 `APP_WIFI_SSID/APP_WIFI_PASSWORD/APP_UPLOAD_HOST` 改为 NVM 参数，不再硬编码。
- 增加连接状态指示与失败原因码上报。
- 增加更细粒度的发送队列统计（丢包、重发、超时）。
