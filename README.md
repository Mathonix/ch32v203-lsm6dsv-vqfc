# CH32V203G6U6 + LSM6DSV + VQF-C

面向 **CH32V203G6U6** 的裸机姿态固件：通过 **I2C** 读取 **LSM6DSV** 六轴 IMU，经 **[dusking1/vqf-c](https://github.com/DusKing1/vqf-c)** 完整 **VQF**（纯 C、无 malloc）输出四元数与欧拉角，并用 **USART1** 打印调试信息。

Short English: Bare-metal firmware for CH32V203G6U6 + LSM6DSV (I2C) + full VQF attitude filter ([dusking1/vqf-c](https://github.com/DusKing1/vqf-c) MIT port of [dlaidig/vqf](https://github.com/dlaidig/vqf)), UART debug at 115200. Self-contained `Platform/` register HAL + GCC `Makefile`; optional MounRiver Studio (MRS) import. **6DOF only** (no magnetometer). **Caution:** full VQF may stress **32 KB Flash / 10 KB SRAM** on G6U6 — check `make size` after linking.

仓库：https://github.com/Mathonix/ch32v203-lsm6dsv-vqfc

---

## 硬件假设 / Hardware

| 项目 | 约定 |
|------|------|
| MCU | **CH32V203G6U6**（QFN28，**32 KB Flash / 10 KB SRAM**）— `Startup/link.ld` 已按此容量配置 |
| 内核系列 | CH32V20x **D6**（与 F6/C6/G6 同启动文件） |
| IMU | **LSM6DSV**，I2C，**WHO_AM_I = 0x70**（ST DS13476 / lsm6dsv-pid） |
| I2C 地址 | 默认 **0x6A**（7-bit，**SA0/SDO = GND**）；SA0 接 Vdd_IO 时为 **0x6B** |
| I2C1 引脚 | **PB6 = SCL，PB7 = SDA**（WCH 常见默认映射，未使能 I2C1 remap） |
| 调试串口 | **USART1 TX = PA9**，115200 8N1（RX=PA10 已配置，本 demo 主要打印） |
| 上拉 | I2C 需外部上拉（典型 4.7 kΩ 至 Vdd_IO）；SDA/SCL 开漏 |

若改用其它引脚，请同步修改 `Platform/platform_ch32v203.c` 并更新本文档。

---

## 目录结构

```
README.md
LICENSE
.gitignore
Makefile
Startup/
  startup_ch32v20x_D6.S
  link.ld                 # 32K Flash / 10K RAM
Platform/
  platform.h              # i2c_write/read, delay_ms, uart_printf
  platform_ch32v203.c     # 寄存器级 RCC/GPIO/I2C1/USART1/SysTick
  ch32v203_regs.h
User/
  main.c
  system_ch32v20x.c/h
  ch32v20x_it.c/h
  ch32v20x_conf.h         # MRS/SPL 兼容占位
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
- ODR **120 Hz**（`ODR_AT_120Hz = 0x6`），FS **±4 g / ±2000 dps**
- **BDU** + **IF_INC**
- 灵敏度：accel **0.122 mg/LSB**，gyro **70 mdps/LSB** → 输出 **m/s²** 与 **rad/s**（VQF 所需 SI 单位）

### Attitude filter: VQF-C (full VQF)
- Vendored from **[DusKing1/vqf-c](https://github.com/DusKing1/vqf-c)** (MIT, Hugo Chiang) — full C port of **[dlaidig/vqf](https://github.com/dlaidig/vqf)** by **Daniel Laidig & Thomas Seel** (Information Fusion 2023).
- API used: `initVqf` / `updateGyr` / `updateAcc` / `getQuat6D`（本工程 **不调用** `updateMag`）
- **6DOF mode**：`initVqf(1/120, 1/120, 5.0f)` — `gyrTs`/`accTs` 对齐 LSM6DSV **120 Hz** ODR；大 `magTs`（上游 README 示例 5.0）且永不调用 `updateMag`
- 单位约定（与原版 VQF 文档一致）：陀螺 **rad/s**，加速度 **m/s²**（驱动已转换）
- 无磁计 → **偏航会漂移**（6DOF 正常现象）
- 欧拉角由 `main.c` 本地四元数辅助函数计算并 UART 打印

### Flash / RAM caution (CH32V203G6U6)
- `vqf.c` 约 **39 KB** 源码，含完整 9D/磁干扰抑制等路径；即使 `--gc-sections` + `-Os`，**32 KB Flash** 仍可能不够。链接后务必 `make size`。
- 静态状态约数百字节～1 KB 量级 BSS（params/coeffs/state），再叠加栈与驱动；**10 KB SRAM** 也需留意。
- 可选瘦身提示（不破坏公开 API）：更大 Flash 型号；或在确认不调用 `updateMag`/`getQuat9D` 时依赖链接器 GC 剔除未引用符号（本仓库未改算法内核）。本地相对上游的小修复见 `Middleware/vqf-c/NOTICE`（`initVqf` 补调 `init_params()`）。

### main 循环
1. 初始化时钟 / UART / I2C / IMU / VQF  
2. WHO_AM_I 失败则打印错误并 **halt**  
3. 约 **8 ms** 周期读 IMU → `updateGyr` → `updateAcc`；约 **每 100 ms** 打印 `getQuat6D` 四元数与 roll/pitch/yaw  

---

## 构建方式

### A. GCC Makefile（推荐先试）

依赖：`riscv-none-elf-gcc`（xPack）或 MRS 自带的 `riscv-none-embed-gcc`。

```bash
make
# 产物：build/firmware.elf|.hex|.bin
make size   # 确认是否超出 32K Flash
```

烧录需 WCH-Link + OpenOCD（或 MRS 下载），`make flash` 仅为示例，请按本机 `openocd`/`wch-riscv.cfg` 调整。

### B. MounRiver Studio

1. 新建 **CH32V203** 工程，芯片选 **CH32V203G6U6**（或同 D6、确认 Flash/RAM）。  
2. 将本仓库 `User/`、`Sensors/`、`Middleware/`、`Platform/` 加入工程；用本仓库 `Startup/link.ld` 替换默认链接脚本（**32K/10K**）。  
3. **可选**：从 MRS pack 加入 WCH SPL；若继续用 `Platform/` 寄存器实现，可不链 SPL 的 I2C/USART 源文件以免重复。  
4. 包含路径加上 `Platform`、`Sensors/lsm6dsv`、`Middleware/vqf-c`。  
5. 编译下载，串口 115200 查看输出。

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
