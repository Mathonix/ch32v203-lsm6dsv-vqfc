# CH32V203G6U6 + LSM6DSV + VQF-C

面向 **CH32V203G6U6** 的裸机姿态固件：通过 **I2C** 读取 **LSM6DSV** 六轴 IMU，经 **[dusking1/vqf-c](https://github.com/DusKing1/vqf-c)** 完整 **VQF**（纯 C、无 malloc）输出四元数与欧拉角，并用 **USART1** 打印调试信息。

Short English: Bare-metal firmware for CH32V203G6U6 + LSM6DSV (I2C) + full VQF attitude filter ([dusking1/vqf-c](https://github.com/DusKing1/vqf-c) MIT port of [dlaidig/vqf](https://github.com/dlaidig/vqf)). **1 kHz** Euler out on **USART1 binary @ 921600** and **CAN1 @ 1 Mbit** (std ID `0x321`). Boot banner still uses printf once. Self-contained `Platform/` register HAL + GCC `Makefile`; optional MRS. **6DOF only**. Full hot path (sample+euler+uart_bin+can_tx + libm) in **32 KB zero-wait** Flash (`< 0x8000`).

仓库：https://github.com/Mathonix/ch32v203-lsm6dsv-vqfc

---

## 硬件假设 / Hardware

| 项目 | 约定 |
|------|------|
| MCU | **CH32V203G6U6**（QFN28） |
| Flash 布局 | **R0WAIT = 32 KB** zero-wait @ `0x00000000`；**NZW ≈ 192 KB** @ `0x00008000`（总 CodeFlash ≈ 224 KB = 32K + 192K） |
| RAM | **10 KB** @ `0x20000000` |
| 内核系列 | CH32V20x **D6**（与 F6/C6/G6 同启动文件） |
| IMU | **LSM6DSV**，I2C，**WHO_AM_I = 0x70**（ST DS13476 / lsm6dsv-pid） |
| I2C 地址 | 默认 **0x6A**（7-bit，**SA0/SDO = GND**）；SA0 接 Vdd_IO 时为 **0x6B** |
| I2C1 引脚 | **PB6 = SCL，PB7 = SDA**（WCH 常见默认映射，未使能 I2C1 remap） |
| 调试串口 | **USART1 TX = PA9**，**921600** 8N1 二进制欧拉流（RX=PA10 已配置）；启动横幅仍用 printf |
| CAN1 | **PA11 = CAN_RX，PA12 = CAN_TX**（默认 AF remap）；需外部收发器；**1 Mbit/s**，标准 ID **`0x321`** |
| USB | **USBD 关闭**（与 CAN 共用 512B SRAM，本工程只开 CAN） |
| 上拉 | I2C 需外部上拉（典型 4.7 kΩ 至 Vdd_IO）；SDA/SCL 开漏；CAN 收发器侧按模块要求 |

若改用其它引脚，请同步修改 `Platform/platform_ch32v203.c` 并更新本文档。

---

## Non-zero-wait Flash (NZW) layout

WCH datasheet note: **advertised Flash bytes = zero-wait R0WAIT only**. For V203 (non-RB), total CodeFlash ≈ **224 KB**; NZW = `224K − R0WAIT`. On **G6U6**, R0WAIT = **32 KB**, so NZW ≈ **192 KB** starting at **`0x00008000`** when FLASH base is `0x00000000` (this project’s `Startup/link.ld`).

### MEMORY regions (`Startup/link.ld`)

| Region | Origin | Length | Role |
|--------|--------|--------|------|
| `FLASH` | `0x00000000` | 32K | Zero-wait R0WAIT |
| `FLASH_NZW` | `0x00008000` | 192K | Non-zero-wait CodeFlash |
| `RAM` | `0x20000000` | 10K | SRAM |

### Section placement

| Output section | Region | Contents |
|----------------|--------|----------|
| `.init` / `.vector` | `FLASH` | Reset trampoline + vector table |
| `.text_zw` | `FLASH` | `handle_reset`, IRQ stubs, **`SystemInit`**, **`.text.hot`** (`vqf_sample_step` / `vqf_run_1khz` / `quat_to_euler_deg`), VQF hot graph, `lsm6dsv_read_*`, `platform_i2c_*`, **`platform_uart_write_bytes` / `platform_uart_send_euler_bin` / `platform_can_send_euler`**, SysTick helpers, soft-float + libm (`sqrt`/`acos`/`sinf`/`cosf`/`asinf`/`atan2f`/`atanf`/kernels) |
| `.text_nzw` | `FLASH_NZW` | Cold/init via `FLASH_NZW` attribute + **cold VQF only** (`initVqf`/`setup`/`resetState`/`updateMag`/setters/mag getters) — claimed **before** `.text_zw` |
| `.text` / `.fini` | `FLASH_NZW` | Default app / remaining libc (printf, `main` shell) |

Convention: **`FLASH_ZW`** → `.text.hot` (`Platform/flash_zw.h`); **`FLASH_NZW`** → `.text_nzw` (`Platform/flash_nzw.h`). Not WCH `.stext`.

### 1 kHz hot-path ZW guarantee

Goal: **1 kHz VQF output** with **every** high-frequency callee in R0WAIT (`addr < 0x8000`).

1. LSM6DSV XL+GY ODR = **true 1000 Hz** via HAODR (`HAODR_CFG.HAODR_SEL=1`, CTRL1/CTRL2=`0x19`). Without HAODR the same ODR code is 960 Hz.
2. `initVqf(1.0f/1000, 1.0f/1000, 5.0f)`.
3. Tight loop body: `FLASH_ZW vqf_sample_step()` → read IMU → `updateGyr` → `updateAcc` → `getQuat6D`, then **Euler → UART binary + CAN TX** every sample inside `vqf_run_1khz()` (all ZW). Optional `#define VQF_UART_ASCII_DEBUG 1` re-enables ~20 Hz printf (NZW; off by default).
4. Verify: `make verify-zw` (or `python3 scripts/verify_zw_hotpath.py build/firmware.elf`) checks required symbols and **jal** targets from `updateGyr`/`updateAcc` (and math helpers) are all `< 0x8000`. Example:

```bash
make PREFIX=riscv64-unknown-elf-
make verify-zw
riscv64-unknown-elf-nm -n build/firmware.elf | egrep 'updateGyr|updateAcc|quatMultiply|vqf_sample'
```

GNU ld **region-list overflow** (`>FLASH FLASH_NZW`) is **not** supported by this toolchain’s `ld` 2.44 (syntax error), so placement is **explicit**.

### FLASH enhance read mode

Before any NZW code runs, `SystemInit` (kept in zero-wait) unlocks Flash and sets **`FLASH_CTLR` bit 24** — the same poke as WCH EVT `FLASH_Enhance_Mode(ENABLE)` in [`ch32v20x_flash.c`](https://github.com/openwch/ch32v20x/blob/main/EVT/EXAM/SRC/Peripheral/src/ch32v20x_flash.c).

```c
FLASH_KEYR = 0x45670123; FLASH_KEYR = 0xCDEF89AB;
FLASH_CTLR |= (1u << 24);  /* enhance read */
FLASH_CTLR |= (1u << 7);   /* re-lock */
```

Public SPL bit headers do not fully document bit 24; the source of truth is WCH’s `FLASH_Enhance_Mode`. If enhance mode were omitted, fetch/execute from NZW may be unreliable — treat on-hardware validation as required.

### C attribute helper

`Platform/flash_nzw.h`:

```c
#define FLASH_NZW        __attribute__((section(".text_nzw")))
#define FLASH_NZW_RODATA __attribute__((section(".rodata_nzw")))
```

Marked cold today: `platform_init`, UART/I2C/CAN init, `platform_uart_printf`, `lsm6dsv_init`, `main`.
Marked hot: `vqf_sample_step`, `vqf_run_1khz`, `quat_to_euler_deg`, `platform_uart_write_bytes`, `platform_uart_send_euler_bin`, `platform_can_send_euler` via `FLASH_ZW`.

---

## 目录结构

```
README.md
LICENSE
.gitignore
Makefile
Startup/
  startup_ch32v20x_D6.S
  link.ld                 # 32K ZW + 192K NZW / 10K RAM
Platform/
  platform.h
  platform_ch32v203.c     # SystemInit enables FLASH enhance read
  ch32v203_regs.h
  flash_nzw.h             # FLASH_NZW / FLASH_NZW_RODATA
  flash_zw.h              # FLASH_ZW → .text.hot (1 kHz loop)
User/
  main.c
  system_ch32v20x.c/h
  ch32v20x_it.c/h
  ch32v20x_conf.h
Sensors/lsm6dsv/
  lsm6dsv.c/h
Middleware/vqf-c/         # vendored dusking1/vqf-c (MIT)
  vqf.c / vqf.h
  LICENSE / NOTICE / README.upstream.md
vendor/
  NOTICE
  mtkos-ch32v203-minimal/ # MIT，参考用薄寄存器头文件（非构建必需）
```

**未完整 vendor WCH SPL**：WCH SDK 带厂商使用声明，不宜整库拷贝。本仓库默认用自包含 `Platform/`；若在 MRS 中开发，从 MRS 芯片包添加 CH32V20x Peripheral 库即可。

---

## 功能概要

### LSM6DSV
- Soft-reset：CTRL1/CTRL2 power-down → CTRL3 `SW_RESET` → 轮询清除（对齐 ST PID）
- ODR **1000 Hz**（HAODR：`HAODR_CFG=0x01`，CTRL1/CTRL2=`0x19` = `OP_MODE_HAODR|ODR_0x9`；同码在非 HAODR 下为 960 Hz）
- FS **±4 g / ±2000 dps**，**BDU** + **IF_INC**
- 灵敏度：accel **0.122 mg/LSB**，gyro **70 mdps/LSB** → 输出 **m/s²** 与 **rad/s**（VQF 所需 SI 单位）
- 宏：`LSM6DSV_ODR_HZ = 1000.0f`

### Attitude filter: VQF-C (full VQF)
- Vendored from **[DusKing1/vqf-c](https://github.com/DusKing1/vqf-c)** (MIT, Hugo Chiang) — full C port of **[dlaidig/vqf](https://github.com/dlaidig/vqf)** by **Daniel Laidig & Thomas Seel** (Information Fusion 2023).
- API used: `initVqf` / `updateGyr` / `updateAcc` / `getQuat6D`（本工程 **不调用** `updateMag`）
- **6DOF mode**：`initVqf(1/1000, 1/1000, 5.0f)` — `gyrTs`/`accTs` 对齐 LSM6DSV **1000 Hz** HAODR；大 `magTs` 且永不调用 `updateMag`
- 单位约定：陀螺 **rad/s**，加速度 **m/s²**（驱动已转换）
- 无磁计 → **偏航会漂移**（6DOF 正常现象）
- 欧拉角（ZYX deg）在 **ZW** 热路径计算（`atan2f`/`asinf` 已链入 `.text_zw`）
- **每采样**同时输出 UART 二进制包 + CAN 帧（1 kHz）；启动时 printf 横幅一次

### UART binary packet @ 921600 (1 kHz)

| Offset | Type | Content |
|--------|------|---------|
| 0–1 | u8×2 | Magic `0xA5 0x5A` |
| 2–3 | u16 LE | Sequence |
| 4–7 | f32 LE | Roll (deg) |
| 8–11 | f32 LE | Pitch (deg) |
| 12–15 | f32 LE | Yaw (deg) |
| 16 | u8 | XOR of bytes 0…15 |

≈ 17 B × 1000 ≈ 17 kB/s — fine at 921600. Quat omitted for bandwidth. Implement: `platform_uart_write_bytes` (poll TXE, no printf).

### CAN frame @ 1 Mbit (1 kHz)

| Field | Layout (8 bytes LE) |
|-------|---------------------|
| Std ID | `PLATFORM_CAN_STD_ID` default **`0x321`** |
| DLC | 8 |
| Data | `roll_i16`, `pitch_i16`, `yaw_i16` (millidegrees), `seq_u16` |

Pins: **PA11=RX, PA12=TX**. Bitrate `#define PLATFORM_CAN_BITRATE 1000000` (APB1 72 MHz → BTR BRP=6, TS1=8, TS2=3, sample ≈75%). TX: non-blocking mailbox0; if busy after ≤2 polls, drop + `platform_can_drop_count++`. **Transceiver required.** USBD left off (SRAM share).

### main / 1 kHz 循环
1. NZW：初始化时钟 / UART(921600) / I2C / CAN / IMU / VQF；打印启动横幅  
2. WHO_AM_I 失败则打印错误并 **halt**  
3. ZW：`vqf_run_1khz` 每 **1 ms**：`vqf_sample_step` → Euler → `platform_uart_send_euler_bin` + `platform_can_send_euler`  

---

## 构建方式

### A. GCC Makefile（推荐先试）

依赖：`riscv-none-elf-gcc`（xPack）或 MRS 自带的 `riscv-none-embed-gcc`，或 Debian/Ubuntu `riscv64-unknown-elf-gcc` + picolibc。

```bash
make PREFIX=riscv64-unknown-elf-
# 产物：build/firmware.elf|.hex|.bin
make size
make verify-zw   # assert hot symbols + jal targets < 0x8000
```

Prefer **ELF/HEX** for programming: the `.bin` spans `0x0000`–end of image and includes a **zero-filled hole** between the end of R0WAIT content and NZW @ `0x8000`.

烧录需 WCH-Link + OpenOCD（或 MRS 下载），`make flash` 仅为示例，请按本机 `openocd`/`wch-riscv.cfg` 调整。

### B. MounRiver Studio

1. 新建 **CH32V203** 工程，芯片选 **CH32V203G6U6**（或同 D6）。  
2. 将本仓库 `User/`、`Sensors/`、`Middleware/`、`Platform/` 加入工程；用本仓库 `Startup/link.ld` 替换默认链接脚本（**32K ZW + 192K NZW / 10K**）。  
3. **可选**：从 MRS pack 加入 WCH SPL；若继续用 `Platform/` 寄存器实现，可不链 SPL 的 I2C/USART 源文件以免重复。  
4. 包含路径加上 `Platform`、`Sensors/lsm6dsv`、`Middleware/vqf-c`。  
5. 编译下载；串口 **921600** 收二进制包，或仅看启动横幅文本。

---

## 第三方说明

见 `vendor/NOTICE` 与 `Middleware/vqf-c/NOTICE`。

| 组件 | 许可 / 来源 |
|------|-------------|
| VQF-C | MIT — [DusKing1/vqf-c](https://github.com/DusKing1/vqf-c)（Hugo Chiang） |
| VQF 算法 | [dlaidig/vqf](https://github.com/dlaidig/vqf) — Laidig & Seel |
| mtkos-ch32v203-minimal | MIT — 寄存器参考 |

LSM6DSV 寄存器与 WHO_AM_I 参考 ST 公开资料（lsm6dsv-pid / 数据手册）。

---

## 许可

本仓库应用代码：MIT（见 `LICENSE`）。`Middleware/vqf-c/` 遵循其上游 MIT（Hugo Chiang）；芯片厂商 SPL 与 ST 驱动头文件各自遵循原许可证。

## 实测体积（本机交叉编译，1 kHz 全热路径 ZW）

工具链：`riscv64-unknown-elf-gcc` 14.2（Debian/Ubuntu apt）+ picolibc，`-Os -fno-math-errno -ffunction-sections -fdata-sections --gc-sections`，`rv32imac_zicsr` / `ilp32`。

| 项 | 数值 |
|---|---|
| Links cleanly? | **Yes**（无 ZW overflow） |
| ODR / outputs | **1000 Hz** HAODR; UART binary + CAN Euler every sample |
| UART baud | **921600** |
| CAN | **1 Mbit/s**, std ID **0x321**, PA11/PA12 |
| FLASH zero-wait used (`.init`+`.vector`+`.text_zw`) | **~24780 B** / 32768 B (~75.6%) |
| FLASH_NZW used (`.text_nzw`+`.text`) | **~15516 B** / 196608 B (~7.9%) |
| Berkeley `text` | **~40296 B** |
| `data` | **8 B** |
| RAM (`data`+`bss`+1 KB stack) | **~1612 B** / 10 KB |
| `make verify-zw` | **OK** — sample/euler/uart_bin/can_tx + `atan2f`/`asinf` **无 jal ≥ 0x8000** |

结论：完整 `dusking1/vqf-c` 1 kHz 热路径 + Euler libm + UART/CAN TX **可装入 32K R0WAIT**（约 8 KB 余量）；冷 init/printf 留在 NZW；`SystemInit` 仍启用 FLASH enhance read。
