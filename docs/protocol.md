# 姿态输出协议（对照 CH32V203G6U6 原理图）

> 原理图 MCU：`CH32V203G6U6`（U1）  
> IMU：`LSM6DSVTR`（U5）SPI  
> CAN 收发器：`SIT1051ATK/3`（U6）  
> 固件分支：`feat/vqf-fixedpoint-rv`，默认定点 VQF-fxp @ **1 kHz**  
> 欧拉：航空航天 **ZYX**（roll / pitch / yaw）  
> 角度单位：**毫度 (millideg)**，`value / 1000.0 = 度`；多数字段为 **小端 (LE)**

---

## 物理连接（原理图）

### 串口（H2）

| H2 | 网名 | MCU |
|---:|---|---|
| 1 | U2_TX | **PA2** |
| 2 | U2_RX | **PA3** |
| 3 | GND | GND |

| 项 | 值 |
|---|---|
| 外设 | USART2 |
| 波特率 | **921600** 8N1 |
| 1 kHz 输出 | TX（MCU → 主机） |
| 启动命令 | RX 可收 `SETALGO`（约 1.5 s 窗口） |

### CAN（H2 + U6）

| H2 | 网名 |
|---:|---|
| 4 | CAN_H |
| 5 | CAN_L |

| 网名 | MCU | 收发器 |
|---|---|---|
| CAN_TX | **PA12** | U6 TXD |
| CAN_RX | **PA11** | U6 RXD |

| 项 | 值 |
|---|---|
| 控制器 | CAN1 Remap1 |
| 波特率 | **1 Mbit/s**（约 75% 采样点） |
| 帧类型 | 标准数据帧 |
| 标准 ID | **`0x321`**（`PLATFORM_CAN_STD_ID`） |
| DLC | **8** |
| 终端电阻 | 板载 **120 Ω**（R4） |
| 发送策略 | 邮箱 0，非阻塞；忙则丢弃并计数 |

> USBD 保持关闭（与 CAN 共享 SRAM）。

### IMU（内部，供对照）

| 网名 | MCU |
|---|---|
| LSM_CS | PA4 |
| LSM_SCK | PA5 |
| LSM_MISO | PA6 |
| LSM_MOSI | PA7 |
| LSM_INT1 | PB0 |
| LSM_INT2 | PB1 |

SPI1 mode 3（CPOL=1, CPHA=1），读寄存器地址 `| 0x80`。

### 其它

| 网名 | MCU | 说明 |
|---|---|---|
| LED | PA9 | 低电平点亮（固件可后加） |
| SWDIO / SWCLK | PA13 / PA14 | H1 调试 |

---

## 1. UART 二进制帧

### 1.1 定点主路径（推荐，VQF-fxp @ 1 kHz）

**总长 11 字节**，魔术字 `A5 5B`。

| 偏移 | 长度 | 类型 | 字段 | 说明 |
|---:|---:|---|---|---|
| 0 | 1 | `u8` | magic0 | 固定 `0xA5` |
| 1 | 1 | `u8` | magic1 | 固定 `0x5B`（与 float 帧 `0x5A` 区分） |
| 2 | 2 | `u16` LE | seq | 帧序号，每包 +1，回绕 |
| 4 | 2 | `i16` LE | roll_mdeg | 横滚，毫度 |
| 6 | 2 | `i16` LE | pitch_mdeg | 俯仰，毫度 |
| 8 | 2 | `i16` LE | yaw_mdeg | 偏航，毫度（无磁力计会漂） |
| 10 | 1 | `u8` | xor8 | `pkt[0] ^ … ^ pkt[9]` |

**布局**

```
Byte:  0    1    2      3      4    5    6    7    8    9    10
      A5   5B   seq_lo seq_hi  R_lo R_hi P_lo P_hi Y_lo Y_hi XOR
```

**校验（接收端）**

```c
uint8_t x = 0;
for (int i = 0; i < 10; i++) x ^= pkt[i];
ok = (x == pkt[10]);
```

**带宽**：11 × 1000 ≈ 11 KB/s @ 921600。

实现：`platform_uart_send_euler_i16()`。

### 1.2 兼容 float 帧（Mahony / 互补滤波路径）

**总长 17 字节**，魔术字 `A5 5A`。

| 偏移 | 长度 | 类型 | 字段 |
|---:|---:|---|---|
| 0–1 | 2 | `u8×2` | `0xA5 0x5A` |
| 2–3 | 2 | `u16` LE | seq |
| 4–7 | 4 | `f32` LE | roll_deg |
| 8–11 | 4 | `f32` LE | pitch_deg |
| 12–15 | 4 | `f32` LE | yaw_deg |
| 16 | 1 | `u8` | xor8（`⊕` 字节 0…15） |

解析时先看 `pkt[1]`：`0x5B` → 11 字节定点帧；`0x5A` → 17 字节浮点帧。

实现：`platform_uart_send_euler_bin()`。

### 1.3 启动控制（文本，非 1 kHz）

上电约 1.5 s 窗口，ASCII 行：

```text
SETALGO n
```

| n | 算法 |
|---|---|
| 0 | VQF-fxp（默认，HOT_RAM） |
| 1 | Mahony（ALGO_RAM） |
| 2 | 互补滤波（ALGO_RAM） |

写入 Flash 标志后需 **复位** 生效。也可用 `scripts/setalgo.py --id n`。

---

## 2. CAN 数据帧（1 kHz）

| 项 | 值 |
|---|---|
| 标准 ID | `0x321` |
| DLC | 8 |

| 偏移 | 长度 | 类型 | 字段 | 说明 |
|---:|---:|---|---|---|
| 0 | 2 | `i16` LE | roll_mdeg | 横滚毫度 |
| 2 | 2 | `i16` LE | pitch_mdeg | 俯仰毫度 |
| 4 | 2 | `i16` LE | yaw_mdeg | 偏航毫度 |
| 6 | 2 | `u16` LE | seq | 与 UART 同拍序号 |

**布局**

```
Byte:  0    1    2    3    4    5    6      7
      R_lo R_hi P_lo P_hi Y_lo Y_hi seq_lo seq_hi
```

**邮箱打包（参考实现）**

```c
TXMDLR = (uint16_t)roll_mdeg | ((uint32_t)(uint16_t)pitch_mdeg << 16);
TXMDHR = (uint16_t)yaw_mdeg  | ((uint32_t)seq << 16);
```

实现：`platform_can_send_euler_i16()`（定点）/ `platform_can_send_euler()`（float→毫度）。

邮箱忙可能丢帧；用 `seq` 与 UART 对账。调试计数：`platform_can_drop_count`。

---

## 3. 同步与单位

- 同一采样节拍：算欧拉后依次发 **UART + CAN**，`seq` 相同。
- 无磁力计时 **yaw 会漂移**。
- 换算：

```c
float deg = (float)mdeg / 1000.0f;
```

---

## 4. 修订记录

| 日期 | 说明 |
|---|---|
| 2026-09-11 | 按 CH32V203G6U6 原理图整理；定点 `A5 5B` 11B + CAN `0x321` 8B |
