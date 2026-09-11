# CH32V203G6U6 + LSM6DSV + multi-algo fusion

面向 **CH32V203G6U6** 的裸机姿态固件：通过 **SPI** 读取 **LSM6DSV** 六轴 IMU，经可切换的 **6DOF** 融合算法输出四元数与欧拉角。默认 **VQF-fxp**（固定小数、VQF 结构；见 `Middleware/vqf-fxp/`）在 **HOT_RAM（SRAM）** 执行（启动时从 ZW Flash LMA 拷贝）；浮点 [dusking1/vqf-c](https://github.com/DusKing1/vqf-c) 源码保留但本分支不链入；可选 **Mahony** / **complementary** 从 NZW 拷入 **ALGO_RAM** 后在 SRAM 执行。算法选择持久化在 Flash 标志位。

> **Schematic note:** 原理图主控为 **AT32F423KCU7-4**；本仓库固件目标为 **CH32V203**，在 AF 允许处对齐同名网络（尤其 **LSM SPI PA4–PA7**）。UART/CAN 在 CH32 上的复用与 AT32 不同，见下表。

Short English: Bare-metal CH32V203G6U6 + LSM6DSV (**SPI** mode 3) with selectable 6DOF fusion (**VQF-fxp** fixed-point default in **HOT_RAM SRAM** on `feat/vqf-fixedpoint-rv`; **Mahony** / **complementary** in **ALGO_RAM**). **1 kHz** Euler as **int16 millideg** on **USART2 binary @ 921600** (magic `A5 5B`) + **CAN1 @ 1 Mbit** (`0x321`). Algo flag in NZW @ `0x37000`.

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
| MAG_SCL/SDA | PB6/PB7 | *(not init)* | Mag I2C out of scope |
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
| `HOT_RAM` | `0x20000000` | 4K | SRAM execute window for VQF-fxp 1 kHz hot path |
| `ALGO_RAM` | `0x20001000` | 4K | SRAM execute window for Mahony + complementary |
| `RAM` | `0x20002000` | 2K | `.data` / `.bss` / stack (1024 B) |

### Section placement

| Output section | Region | Contents |
|----------------|--------|----------|
| `.init` / `.vector` | `FLASH` | Reset trampoline + vector table |
| `.text_zw` | `FLASH` | `handle_reset`, `SystemInit`, `SysTick_Handler`, `memcpy`/`memset`, **`__divdi3` / `__udivdi3` / `__umoddi3`** (Option B) — **no soft-float/libm** |
| `.text.hot_ram` | `HOT_RAM` AT>`FLASH` | VQF-fxp 1 kHz business: `fusion_run_1khz_fxp`, `vqfx_*`, quat helpers, SPI Q16, UART `A5 5B` + CAN i16, `platform_millis` |
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

**Honesty label:** this is a **VQF-structured fixed-point** 6DOF filter inspired by [dusking1/vqf-c](https://github.com/DusKing1/vqf-c) (strapdown `gyrQuat` + inclination `accQuat` + light bias), **not** a bit-exact full Laidig VQF port (no Kalman `biasP` / rest-LP / mag path). Sources: `Middleware/vqf-fxp/`.

### Q-format

| Quantity | Format | Scale |
|----------|--------|-------|
| Quaternion wxyz | **Q30** | `1.0 = 1<<30` |
| Gyro rates | **Q16** rad/s | `1.0 rad/s = 65536` |
| Accel | **Q16** m/s² | `1.0 m/s² = 65536` |
| Euler output | int32 / int16 **millideg** | `1000 ≡ 1°` |

### LSM6DSV raw → Q16 (no float on hot path)

| Axis | FS | LSB weight (SI) | Integer scale |
|------|----|-----------------|---------------|
| Acc (±4 g) | 0.122 mg/LSB | ≈0.0011964 m/s² | **×78** → Q16 |
| Gyr (±2000 dps) | 70 mdps/LSB | ≈0.0012217 rad/s | **×80** → Q16 |

API: `lsm6dsv_read_acc_gyr_fxp()` in `Sensors/lsm6dsv/` (FLASH_ZW).

### RISC-V M acceleration (QingKe V4B / `rv32imac`)

- Compile: `-march=rv32imac_zicsr` (unchanged)
- Q-format MACs use `int64_t` products → GCC emits **`mul` / `mulh`**
- 32-bit quotients use hardware **`div`**; remaining int64 quotients use ZW **`__divdi3`**
- Hot path: **no** soft-float (`__*sf3`) and **no** libm (`sinf`/`cosf`/`atan2f`/`asinf`/`sqrt`)
- Small-angle Taylor sin/cos for gyro δq; fixed-point atan2/asin → millideg for Euler

### UART / CAN @ 1 kHz

| Bus | Format |
|-----|--------|
| USART2 | **11-byte** packet: magic **`A5 5B`**, `seq` u16 LE, roll/pitch/yaw **int16 millideg** LE, xor8 |
| CAN1 `0x321` | 8 bytes: roll/pitch/yaw int16 millideg + seq (unchanged packing) |

Legacy float UART magic `A5 5A` (17 bytes) remains for Mahony/Comp NZW path only.

### ZW size (before → after on this branch)

| Metric | Float VQF (`main` @ fbce22b) | VQF-fxp (this branch) |
|--------|------------------------------|------------------------|
| ZW (init+vector+text_zw) | **~24364 B** (~74% of 32K) | **~6548 B** (~20% of 32K) |
| Soft-float / libm in ZW | yes (`__addsf3`, `atan2f`, …) | **none** (soft-float stays NZW for Mahony/Comp) |

`make verify-zw` enforces fxp hot symbols in **HOT_RAM** and soft-float **not** in ZW.

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
Middleware/vqf-fxp/       # fixed-point VQF-structured 6DOF (default hot path)
  vqf_fxp.c / vqf_fxp.h / README.md
Middleware/vqf-c/         # vendored dusking1/vqf-c (MIT) — kept, not linked on this branch
  vqf.c / vqf.h
Middleware/fusion/        # unified API + Mahony + complementary
  fusion.h / fusion_vqf.c / fusion_select.c
  mahony.c/h / complementary.c/h
Platform/algo_cfg.h/.c    # Flash flag read/write
scripts/setalgo.py        # host patch / OpenOCD helpers
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
- ODR: HAODR true **1000 Hz** (`HAODR_CFG=0x01`, CTRL1/CTRL2=`0x19`)
- FS: ±4 g / ±2000 dps
- 宏：`LSM6DSV_ODR_HZ = 1000.0f`
- Hot path: `lsm6dsv_read_acc_gyr` + `platform_spi_xfer` / `platform_lsm_cs` in **FLASH_ZW**

### Attitude filters (selectable)
- **VQF** (default): vendored **[DusKing1/vqf-c](https://github.com/DusKing1/vqf-c)** — `initVqf(1/1000,1/1000,5.0)` 6DOF, no `updateMag`; runs in **ZW**
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

### main / 1 kHz 循环
1. NZW：platform init → read algo flag → `fusion_boot_select` → boot log → optional `SETALGO` window  
2. IMU init；`fusion_active->init(1000)`  
3. ZW：`fusion_run_1khz` 每 **1 ms**：IMU → fusion update/get_quat → Euler → UART + CAN  

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

## 实测体积（本机交叉编译，multi-algo）

工具链：`riscv64-unknown-elf-gcc` 14.2 + picolibc，`-Os`，`rv32imac_zicsr` / `ilp32`。

| 项 | 数值 |
|---|---|
| Links cleanly? | **Yes** |
| ODR / outputs | **1000 Hz** HAODR; UART binary + CAN Euler every sample |
| Algo flag | **`0x00037000`** (FPEC `0x08037000`), magic `ALGO` |
| HOT_RAM reserved | **4096 B** @ `0x20000000` |
| `.text.hot_ram` used | **3136 B** (VQF-fxp business; `__divdi3` family stays ZW) |
| ALGO_RAM reserved | **4096 B** @ `0x20001000` |
| `.algo_ram` used | **3144 B** (Mahony + Comp) |
| RAM region | **2048 B** @ `0x20002000` — `.data` 24 + `.bss` 120 + stack 1024; **free ≈880 B** |
| FLASH ZW (init+vector+text_zw) | **3440 B** / 32768 B |
| `make verify-zw` | **OK** — VQF hot path HOT_RAM `0x2000xxxx`; Mahony/Comp in ALGO_RAM |

结论：默认 VQF-fxp 1 kHz 热路径在 **HOT_RAM（SRAM）** 执行；`__divdi3` 族保留 ZW；交替算法在 ALGO_RAM；1 kHz UART+CAN 路径保持。
