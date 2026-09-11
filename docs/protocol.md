# 姿态输出协议（对照 CH32V203G6U6 原理图）

> 原理图 MCU：`CH32V203G6U6`（U1）  
> IMU：`LSM6DSVTR`（U5）SPI  
> CAN 收发器：`SIT1051ATK/3`（U6）  
> 固件分支：`main`（浮点 VQF 默认 @ **1 kHz**）  
> 欧拉：航空航天 **ZYX**；多数字段 **小端 (LE)**

定点优化分支见 `feat/vqf-fixedpoint-rv`（UART 魔术字 `A5 5B` 毫度包）。

---

## 多算法切换

| algo_id | 算法 | 执行位置 |
|---:|---|---|
| **0**（默认 / 标志无效） | VQF（dusking1/vqf-c） | 零等待 Flash |
| **1** | Mahony 6DOF | NZW → **ALGO_RAM**（SRAM） |
| **2** | 互补滤波 6DOF | NZW → **ALGO_RAM**（SRAM） |

| 项 | 值 |
|---|---|
| Flash 标志地址 | CPU `0x00037000` / FPEC `0x08037000` |
| 布局 | magic `0x414C474F` + version + algo_id + checksum |
| 修改方式 | 上电串口 `SETALGO n\n` 后复位，或 `scripts/setalgo.py --id n` |

---

## 物理连接（原理图）

### 串口（H2）

| H2 | 网名 | MCU |
|---:|---|---|
| 1 | U2_TX | **PA2** |
| 2 | U2_RX | **PA3** |
| 3 | GND | GND |

USART2，**921600 8N1**，1 kHz 二进制 TX；启动窗口 RX 收 `SETALGO`。

### CAN（H2 + U6）

| H2 | 网名 |
|---:|---|
| 4 | CAN_H |
| 5 | CAN_L |

| 网名 | MCU |
|---|---|
| CAN_TX | **PA12** |
| CAN_RX | **PA11** |

CAN1 Remap1，**1 Mbit/s**，标准 ID **`0x321`**，DLC=8，板载 120Ω。

### IMU

`PA4=CS, PA5=SCK, PA6=MISO, PA7=MOSI`，`PB0=INT1, PB1=INT2`；SPI1 mode 3。

---

## 1. UART 二进制帧（`main` 浮点主路径，1 kHz）

**总长 17 字节**，魔术字 `A5 5A`。

| 偏移 | 长度 | 类型 | 字段 |
|---:|---:|---|---|
| 0–1 | 2 | `u8×2` | `0xA5 0x5A` |
| 2–3 | 2 | `u16` LE | seq |
| 4–7 | 4 | `f32` LE | roll_deg |
| 8–11 | 4 | `f32` LE | pitch_deg |
| 12–15 | 4 | `f32` LE | yaw_deg |
| 16 | 1 | `u8` | xor8（`⊕` 字节 0…15） |

实现：`platform_uart_send_euler_bin()`。

> `feat/vqf-fixedpoint-rv` 使用 11 字节 `A5 5B` + int16 毫度；可用 `pkt[1]` 区分。

### 启动命令

```text
SETALGO n\n   # n=0|1|2，写 Flash 后复位
```

---

## 2. CAN 数据帧（1 kHz）

| 项 | 值 |
|---|---|
| ID | `0x321` |
| DLC | 8 |

| 偏移 | 类型 | 字段 |
|---:|---|---|
| 0 | i16 LE | roll_mdeg（由度×1000） |
| 2 | i16 LE | pitch_mdeg |
| 4 | i16 LE | yaw_mdeg |
| 6 | u16 LE | seq（与 UART 同拍） |

实现：`platform_can_send_euler()`。

---

## 3. 同步

同一采样：欧拉 → UART + CAN，共享 `seq`。无磁力计时 yaw 会漂。
