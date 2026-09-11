# CH32V203G6U6 + LSM6DSV + multi-algo fusion

面向 **CH32V203G6U6** 的裸机姿态固件：通过 **SPI** 读取 **LSM6DSV** 六轴 IMU，经可切换的 **6DOF** 融合算法输出四元数与欧拉角。默认 **Full VQF fixed**（`Middleware/vqf_fixed/`，设计分域定点 6D）在 **HOT_RAM（SRAM）** 执行；`vqf-fxp` 仅为 deprecated Q16 包装（启动时从 ZW Flash LMA 拷贝）；浮点 [dusking1/vqf-c](https://github.com/DusKing1/vqf-c) 源码保留但本分支不链入；可选 **Mahony** / **complementary** 从 NZW 拷入 **ALGO_RAM** 后在 SRAM 执行。算法选择持久化在 Flash 标志位。

> **Schematic note:** 原理图主控为 **AT32F423KCU7-4**；本仓库固件目标为 **CH32V203**，在 AF 允许处对齐同名网络（尤其 **LSM SPI PA4–PA7**）。UART/CAN 在 CH32 上的复用与 AT32 不同，见下表。

Short English: Bare-metal CH32V203G6U6 + LSM6DSV (**SPI** mode 3) with selectable 6DOF fusion (**vqf_fixed** Full 6D fixed-point default in **HOT_RAM SRAM** on `feat/vqf-fixedpoint-rv`; **Mahony** / **complementary** in **ALGO_RAM**). LSM **GY HAODR 4 kHz / XL 1 kHz** async; **vqf_fixed_update_gyr @ 4 kHz**, **update_acc + UART/CAN Euler @ 1 kHz** (int16 millideg, USART2 `A5 5B` @ 921600 + CAN1 `0x321` @ 1 Mbit). Algo flag in NZW @ `0x37000`. Mag/9D **off** (`VQF_FIXED_ENABLE_MAG=0`).

通信帧格式见 [docs/protocol.md](docs/protocol.md)（UART / CAN，对照 CH32V203G6U6 原理图）。


仓库：https://github.com/Mathonix/ch32v203-lsm6dsv-vqfc

---

## 硬件假设 / Hardware

| 项目 | 约定 |
|------|------|
| MCU (FW) | **CH32V203G6U6**（QFN28）；原理图 MCU 为 **AT32F423KCU7-4** |
| Flash 布局 | **R0WAIT = 32 KB** ZW @ `0x00000000`；**NZW code 188 KB** @ `0x00008000`；**algo_cfg 4 KB** @ `0x00037000`（总 CodeFlash ≈ 224 KB） |
| RAM | **10 KB** total：`HOT_RAM` **4 KB** @ `0x20000000` + `ALGO_RAM` **4 KB** @ `0x20001000` + **2 KB** data/bss/stack @ `0x20002000` |
| 内核系列 | CH32V20x **D6**（与 F6/C6/G6 同启动文件） |
| IMU | **LSM6DSV**，**SPI**，**WHO_AM_I = 0x70**（ST DS13476 / lsm6dsv-pid） |
| SPI | **Mode 3** (CPOL=1 CPHA=1)；读寄存器地址 **OR 0x80**；软 CS |
| USB | **USBD 关闭**（与 CAN 共用 512B SRAM，本工程只开 CAN） |

### Pin / net table (schematic AT32 vs this CH32 FW)

| Net | Schematic (AT32F423) | This CH32V203 FW | Notes |
|-----|----------------------|------------------|-------|
| LSM_CS | PA4 | **PA4** GPIO SW CS (active low) | Match |
| LSM_SCK | PA5 | **PA5** SPI1_SCK | Match |
| LSM_MISO | PA6 | **PA6** SPI1_MISO | Match |
| LSM_MOSI | PA7 | **PA7** SPI1_MOSI | Match |
| LSM_INT1 | PB0 | PB0 input (unused) | Match pin; unused in code |
| LSM_INT2 | PB1 | PB1 input (unused) | Match pin; unused in code |
| UART_TX | PA0 (USART4) | **PA2** USART2_TX | CH32 RM: PA0=USART2_CTS only — no TX AF |
| UART_RX | PA1 (USART4) | **PA3** USART2_RX | CH32 RM: PA1=USART2_RTS only — no RX AF |
| CAN_RX | PA2 (CAN2) | **PA11** CAN1 Remap1 | CH32 CAN remaps: PA11/12, PB8/9, PD0/1 only |
| CAN_TX | PA3 (CAN2) | **PA12** CAN1 Remap1 | No CAN AF on PA2/PA3 |
| MAG_SCL/SDA | PB6/PB7 | *(not init)* | IST8310 stub API only; 9D off (HOT_RAM/budget) |
| USB_DM/DP | PA11/PA12 | USBD off; pins used by CAN Remap1 | |

UART: **USART2 @ 921600** 8N1 binary Euler + boot printf. CAN: **1 Mbit/s**, std ID **`0x321`**, transceiver required.

若改用其它引脚，请同步修改 `Platform/platform_ch32v203.c` 并更新本文档。

---

## Non-zero-wait Flash (NZW) layout

WCH datasheet note: **advertised Flash bytes = zero-wait R0WAIT only**. For V203 (non-RB), total CodeFlash ≈ **224 KB**; NZW = `224K − R0WAIT`. On **G6U6**, R0WAIT = **32 KB**, so NZW ≈ **192 KB** starting at **`0x00008000`** when FLASH base is `0x00000000` (this project’s `Startup/link.ld`).

### MEMORY regions (`Startup/link.ld`)

| Region | Origin | Length | Role |
|--------|--------|--------|------|
| `FLASH` | `0x00000000` | 32K | Zero-wait R0WAIT (boot + `__divdi3` + HOT_RAM LMA) |
| `FLASH_NZW` | `0x00008000` | 188K | Non-zero-wait CodeFlash (code + `.algo_ram` LMA) |
| `ALGO_CFG` | `0x00037000` | 4K | Persistent algo select flag (NOLOAD; programmed at runtime) |
| `HOT_RAM` | `0x20000000` | 4K | SRAM execute window for vqf_fixed 4k/1k hot path |
| `ALGO_RAM` | `0x20001000` | 4K | SRAM execute window for Mahony + complementary |
| `RAM` | `0x20002000` | 2K | `.data` / `.bss` / stack (1024 B) |

### Section placement

| Output section | Region | Contents |
|----------------|--------|----------|
| `.init` / `.vector` | `FLASH` | Reset trampoline + vector table |
| `.text_zw` | `FLASH` | `handle_reset`, `SystemInit`, `SysTick_Handler`, `memcpy`/`memset`, **`__divdi3` / `__udivdi3` / `__umoddi3`** (Option B) — **no soft-float/libm** |
| `.text.hot_ram` | `HOT_RAM` AT>`FLASH` | vqf_fixed 4k/1k: `fusion_run_1khz`→4k1k body, `vqf_fixed_update_*`, LSM status/gyr/acc fixed reads, UART `A5 5B` + CAN i16 |
| `.text_nzw` | `FLASH_NZW` | Cold/init + cold VQF (`vqfx_init`/float helpers) |
| `.text` / `.fini` | `FLASH_NZW` | `main`, printf, `algo_cfg_*`, remaining libc |
| `.algo_ram` | `ALGO_RAM` AT>`FLASH_NZW` | Mahony + complementary code (startup memcpy LMA→VMA) |
| `.algo_cfg` | `ALGO_CFG` NOLOAD | Flag slot @ `0x37000` |

Convention: **`FLASH_ZW`** → `.text.hot_ram` (SRAM VMA); **`FLASH_NZW`** → `.text_nzw`; algo code → `.algo_text.*` (VMA=ALGO_RAM). Startup copies HOT_RAM then ALGO_RAM LMA→VMA before `main`.

### 1 kHz hot-path SRAM guarantee

Goal: **1 kHz VQF-fxp output** with business callees in **HOT_RAM** (`0x2000xxxx`). Int64 div helpers may stay in ZW Flash (Option B).

1. LSM6DSV XL+GY ODR = **true 1000 Hz** via HAODR (`HAODR_CFG.HAODR_SEL=1`, CTRL1/CTRL2=`0x19`). Without HAODR the same ODR code is 960 Hz.
2. Tight loop (ALGO_VQF): HOT_RAM `fusion_run_1khz_fxp()` → `lsm6dsv_read_acc_gyr_fxp` → `vqfx_update_gyr/acc` → `vqfx_get_euler_mdeg` → UART `A5 5B` + CAN. Mahony/Comp use NZW float loop + ALGO_RAM.
3. Verify: `make verify-zw` checks VQF-fxp hot symbols in HOT_RAM, soft-float **not** in ZW, Mahony/Comp in ALGO_RAM. Example:

```bash
make PREFIX=riscv64-unknown-elf-
make verify-zw
riscv64-unknown-elf-nm -n build/firmware.elf | egrep 'vqfx_|fusion_run|mahony_|__divdi3'
```

GNU ld **region-list overflow** (`>FLASH FLASH_NZW`) is **not** supported by this toolchain’s `ld` 2.44 (syntax error), so placement is **explicit**.


## Fixed-point VQF (`feat/vqf-fixedpoint-rv`)

**Honesty label:** **Full 6D fixed VQF** wired at design rates (GY 4 kHz / XL 1 kHz). Sources: `Middleware/vqf_fixed/`. Legacy `Middleware/vqf-fxp/` is a **deprecated Q16 wrapper**.

| Done | Remaining / deferred |
|------|----------------------|
| Float VQF-C memory fixes (`FIXES.md`) | Mag / CORDIC / 9D — **off** (`VQF_FIXED_ENABLE_MAG=0`); IST8310 stub only |
| Math / quat / DF-I biquad+residue / scaled LDLT | Real-board cycle map (§19.23) — needs HW |
| updateGyr 4th-order poly + rest gyr LP @ 4 kHz coeffs | Dynamic tau setters (intentionally unsupported) |
| updateAcc LP + inclination + rest + bias Kalman | Motion Kalman vs float still **>0.02°** RMS (see replay) |
| LSM HAODR **GY=4 kHz / XL=1 kHz**; STATUS poll; UART/CAN @ 1 kHz only | INT1/INT2 / FIFO IRQ not used (polling) |
| Host replay `tools/vqf_fixed_replay/` (real float+fixed link) | — |
| `scripts/gen_vqf_fixed_coeffs.py` → `coeffs_{1k1k,4k1k}.h`; `gyr_hz>=3500` selects 4k | — |

**Replay honesty (host `make replay`):** static (0 gyr, +1 g) quat RMS **0.000°** (PASS &lt;0.02°). Mild synthetic motion 10 s quat RMS **≈0.48°** (max ≈1.36°) — **below** design target; bias-Kalman / IIR quantization not Laidig-parity yet. Do not claim bit-exact float match on dynamic data.

### Domains (design)

| Quantity | Format | Notes |
|----------|--------|-------|
| Quaternion / R / unit | **F30** | `1.0 = 1<<30` |
| Angle | **F28** | mag path later |
| Gyro | **F25** rad/s | |
| Acc | **F27** g | not m/s² |
| Bias | **F29** rad/s | clip ±2 °/s |
| P | **F18** | W as U64/F8 |
| IIR coeff | **F30** | PC-precomputed |

Euler output remains int32/int16 **millideg**.

### LSM6DSV raw → F25/F27 (no float on hot path)

| Axis | FS | Integer scale |
|------|----|---------------|
| Acc (±4 g) | 0.122 mg/LSB | **×16375** → g F27 |
| Gyr (±2000 dps) | 70 mdps/LSB | **×40993** → rad/s F25 |

API: `lsm6dsv_read_acc_gyr_fixed()` (FLASH_ZW). Legacy `*_fxp` Q16 retained for wrappers.

### RISC-V M acceleration (QingKe V4B / `rv32imac`)

- Compile: `-march=rv32imac_zicsr` (unchanged)
- Q-format MACs use `int64_t` products → GCC emits **`mul` / `mulh`**
- 32-bit quotients use hardware **`div`**; remaining int64 quotients use ZW **`__divdi3`**
- Hot path: **no** soft-float (`__*sf3`) and **no** libm (`sinf`/`cosf`/`atan2f`/`asinf`/`sqrt`)
- Small-angle Taylor sin/cos for gyro δq; fixed-point atan2/asin → millideg for Euler

### UART / CAN @ 1 kHz only

Attitude packets are emitted on **accel cadence (1 kHz)**, not at 4 kHz gyro rate.

| Bus | Format |
|-----|--------|
| USART2 | **11-byte** packet: magic **`A5 5B`**, `seq` u16 LE, roll/pitch/yaw **int16 millideg** LE, xor8 |
| CAN1 `0x321` | 8 bytes: roll/pitch/yaw int16 millideg + seq (unchanged packing) |

Legacy float UART magic `A5 5A` (17 bytes) remains for Mahony/Comp NZW path only.

### Mag / 9D status

**Off.** `VQF_FIXED_ENABLE_MAG=0`. After 4 kHz wiring, HOT_RAM ≈ **48%** and ZW ≈ **30%** — enough for 6D, but mag CORDIC + disturbance rejection + I2C driver would consume the remaining HOT_RAM headroom and was deferred per design §19.24 until motion replay is closer to 0.02°. Schematic IST8310 on **PB6/PB7** has stub headers in `Sensors/ist8310/` (not linked). API stub: `vqf_fixed_update_mag_f18` under `#if VQF_FIXED_ENABLE_MAG`.

### ZW / HOT_RAM sizes (local `riscv64-unknown-elf` build)

| Metric | Value |
|--------|-------|
| ZW (init+vector+text_zw) | **~9676 B** / 32768 (~29.5%) |
| HOT_RAM (`.text.hot_ram`) | **~1948 B** / 4096 (~47.6%) |
| Soft-float / libm in ZW | **none** (NZW only for Mahony/Comp) |
| `make verify-zw` | **OK** |

`make verify-zw` enforces vqf_fixed updates + LSM status/gyr/acc fixed reads in **HOT_RAM** and soft-float **not** in ZW. Host: `make replay`.

## Multi-algorithm selection

### Algo IDs (`u32`)

| ID | Name | Execute from |
|----|------|--------------|
| `0` / `ALGO_VQF` | VQF-fxp (default) | HOT_RAM (SRAM execute after boot copy) |
| `1` / `ALGO_MAHONY` | Mahony AHRS 6DOF | ALGO_RAM (SRAM) after boot copy |
| `2` / `ALGO_COMPLEMENTARY` | Complementary filter 6DOF | ALGO_RAM (SRAM) after boot copy |
| invalid / erased | treated as VQF | ZW |

### Flash flag (`algo_cfg_t` @ `0x00037000`)

| Field | Type | Value |
|-------|------|-------|
| magic | u32 | `0x414C474F` (`'ALGO'`) |
| version | u32 | `1` |
| algo_id | u32 | 0 / 1 / 2 |
| checksum | u32 | `magic ^ version ^ algo_id` |

- **CPU map / read**: `ALGO_CFG_ADDR_CPU = 0x00037000`
- **FPEC program/erase**: `ALGO_CFG_ADDR_FPEC = 0x08037000` (WCH Flash alias)
- Erase granularity: standard **4 KB** page
- APIs: `algo_cfg_read()` / `algo_cfg_write()` in `Platform/algo_cfg.c` (NZW)

### How to change the flag

1. **UART** (boot window ~1.5 s): send `SETALGO n\n` with `n=0|1|2`, then **reset** the MCU.
2. **Host patch** (before flashing a `.bin` that spans to `0x37000`):
   ```bash
   python3 scripts/setalgo.py --id 1 build/firmware.bin
   python3 scripts/setalgo.py --id 0 --print-openocd   # mww helpers
   ```
3. Prefer programming via ELF + separately poking the flag page (OpenOCD / WCH-Link), because `.algo_cfg` is **NOLOAD** and is not part of the normal image payload.

### SRAM execute note

Mahony + complementary are compiled into `.algo_text.*` with **VMA = ALGO_RAM** @ `0x20001000`, **LMA = FLASH_NZW**. Startup copies `_salgo_ram_lma` → `[_salgo_ram, _ealgo_ram)` (both algos, ~3.2 KB). Function pointers in `fusion_algo_t` hold **RAM addresses**. VQF-fxp hot path is copied into **HOT_RAM** @ `0x20000000` (`_shot_ram_lma` → `[_shot_ram, _ehot_ram)`).

### Unified fusion API (`Middleware/fusion/`)

```c
typedef struct {
  void (*init)(float sample_hz);
  void (*update)(const float gyr[3], const float acc[3], float dt); /* rad/s, m/s2 */
  void (*get_quat)(float q[4]); /* wxyz */
} fusion_algo_t;
```

Boot: `fusion_boot_select()` reads the flag and sets `fusion_active`.

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

Marked cold today: `platform_init`, UART/SPI/CAN init, `platform_uart_printf`, `lsm6dsv_init`, `main`.
Marked hot: `fusion_sample_step`, `fusion_run_1khz`, `quat_to_euler_deg`, VQF wrappers, `platform_uart_write_bytes`, `platform_uart_send_euler_bin`, `platform_can_send_euler` via `FLASH_ZW`.

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
  flash_zw.h              # FLASH_ZW → .text.hot_ram (HOT_RAM SRAM)
User/
  main.c
  system_ch32v20x.c/h
  ch32v20x_it.c/h
  ch32v20x_conf.h
Sensors/lsm6dsv/
  lsm6dsv.c/h
Middleware/vqf_fixed/     # Full VQF fixed 6D (default HOT_RAM path)
Middleware/vqf-fxp/       # deprecated Q16 wrapper
Middleware/vqf-c/         # float baseline for host replay (not linked in FW)
Middleware/fusion/        # unified API + Mahony + complementary
Sensors/ist8310/          # mag stub (unlinked; VQF_FIXED_ENABLE_MAG=0)
tools/vqf_fixed_replay/   # host float vs fixed RMS harness
Platform/algo_cfg.h/.c    # Flash flag read/write
scripts/gen_vqf_fixed_coeffs.py
scripts/setalgo.py / verify_zw_hotpath.py
vendor/
  NOTICE
  mtkos-ch32v203-minimal/ # MIT，参考用薄寄存器头文件（非构建必需）
```

**未完整 vendor WCH SPL**：WCH SDK 带厂商使用声明，不宜整库拷贝。本仓库默认用自包含 `Platform/`；若在 MRS 中开发，从 MRS 芯片包添加 CH32V20x Peripheral 库即可。

---

## 功能概要

### LSM6DSV (SPI)

- Bus: **SPI1** mode 3, soft CS **PA4**, SCK/MISO/MOSI **PA5/PA6/PA7**
- Read protocol: address byte with **MSB=1** (`reg | 0x80`)
- WHO_AM_I = **0x70**
- ODR: HAODR mode 1 — **GY 4 kHz** (`CTRL2=0x1B`), **XL 1 kHz** (`CTRL1=0x19`), `HAODR_CFG=0x01`
- FS: ±4 g / ±2000 dps
- 宏：`LSM6DSV_GYR_ODR_HZ=4000`, `LSM6DSV_ACC_ODR_HZ=1000`, `LSM6DSV_ODR_HZ=1000.0f` (Mahony/UART cadence)
- Hot path: `lsm6dsv_read_status` / `read_gyr_fixed` / `read_acc_fixed` + SPI CS in **FLASH_ZW**

### Attitude filters (selectable)
- **VQF** (default): **`vqf_fixed`** Full 6D @ GY 4 kHz / XL 1 kHz in **HOT_RAM**; float `vqf-c` kept for host replay only (not linked in FW)
- **Mahony 6DOF**: compact AHRS, default Kp=1.0 / Ki=0.0; runs in **ALGO_RAM**
- **Complementary 6DOF**: gyro integrate + accel tilt (α≈0.02 @ 1 kHz); runs in **ALGO_RAM**
- Units: gyro **rad/s**, accel **m/s²**; quat **wxyz**; Euler ZYX deg in ZW
- No magnetometer → **yaw drifts** (expected for 6DOF)
- Every sample: UART binary + CAN Euler @ 1 kHz

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

Pins (CH32 Remap1): **PA11=RX, PA12=TX** (schematic AT32 CAN is PA2/PA3 — not available on CH32). Bitrate `#define PLATFORM_CAN_BITRATE 1000000` (APB1 72 MHz → BTR BRP=6, TS1=8, TS2=3, sample ≈75%). TX: non-blocking mailbox0; if busy after ≤2 polls, drop + `platform_can_drop_count++`. **Transceiver required.** USBD left off (SRAM share).

### main / 4k–1k 循环
1. NZW：platform init → read algo flag → `fusion_boot_select` → boot log → optional `SETALGO` window  
2. IMU init (HAODR 4k/1k)；`fusion_active->init` → vqf_fixed `{4000,1000}`  
3. HOT_RAM：`fusion_run_1khz` → STATUS poll → gyr@4kHz / acc+UART+CAN@1kHz  

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
2. 将本仓库 `User/`、`Sensors/`、`Middleware/`、`Platform/` 加入工程；用本仓库 `Startup/link.ld`（**32K ZW + 188K NZW + 4K algo_cfg / HOT_RAM 4K + ALGO_RAM 4K + RAM 2K**）。  
3. **可选**：从 MRS pack 加入 WCH SPL；若继续用 `Platform/` 可不链 SPL 的 SPI/USART。  
4. 包含路径加上 `Platform`、`Sensors/lsm6dsv`、`Middleware/vqf-c`、`Middleware/fusion`。  
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

## 实测体积（本机交叉编译，multi-algo + vqf_fixed 4k/1k）

工具链：`riscv64-unknown-elf-gcc` + picolibc，`-Os`，`rv32imac_zicsr` / `ilp32`。

| 项 | 数值 |
|---|---|
| Links cleanly? | **Yes** |
| ODR / outputs | GY **4 kHz** / XL **1 kHz** HAODR; UART+CAN Euler @ **1 kHz** |
| Algo flag | **`0x00037000`** (FPEC `0x08037000`), magic `ALGO` |
| HOT_RAM reserved | **4096 B** @ `0x20000000` |
| `.text.hot_ram` used | **~1948 B** (~47.6%; `__divdi3` family stays ZW) |
| ALGO_RAM reserved | **4096 B** @ `0x20001000` |
| `.algo_ram` used | **3144 B** (Mahony + Comp) |
| RAM region | **2048 B** @ `0x20002000` — `.data` 24 + `.bss` + stack 768 |
| FLASH ZW (init+vector+text_zw) | **~9676 B** / 32768 B (~29.5%) |
| Mag / 9D | **Off** (stub API + IST8310 headers only) |
| Host replay | static RMS **0.000°**; motion RMS **≈0.48°** (target 0.02° not met) |
| `make verify-zw` | **OK** — vqf_fixed + LSM fixed reads in HOT_RAM |

结论：默认 **vqf_fixed** 4k/1k 热路径在 **HOT_RAM**；UART/CAN 仅 1 kHz；mag 关闭；浮点回放未达 0.02° 动态目标（诚实记录）。

### Board-only gaps (cannot close in CI)

- Real SPI LSM6DSV HAODR 4k/1k + cycle occupancy on 144 MHz silicon
- INT1/INT2 or FIFO watermark IRQ instead of STATUS poll
- IST8310 I2C bring-up on PB6/PB7 when enabling 9D
- On-air UART/CAN packet validation with transceiver
