# CH32V203G6U6 + LSM6DSV + VQF-C — GCC Makefile
# Toolchain: xPack / MRS riscv-none-elf-gcc or riscv-none-embed-gcc
#
#   make            # build build/firmware.elf
#   make flash      # optional: openocd + wch-link (adjust OPENOCD)
#
# MRS path: create a CH32V203 project, add User/Sensors/Middleware/Platform
# sources, use Startup/link.ld (32K ZW + 192K NZW / 10K RAM). SPL not required for this Makefile.
#
# Flash: G6U6 R0WAIT=32KB zero-wait + ~192KB non-zero-wait (total CodeFlash 224KB).
# Startup/link.ld places vectors/reset/SystemInit + FULL VQF 1 kHz hot call graph
# (updateGyr/updateAcc + callees + soft-float/libm) in FLASH (ZW < 0x8000).
# Cold VQF (init/mag/setters) + printf/euler in FLASH_NZW @ 0x8000.

TARGET   ?= firmware
BUILD    ?= build

# Prefer xPack / MRS names, then Debian/Ubuntu riscv64-unknown-elf
PREFIX   ?= riscv-none-elf-
ifeq ($(shell which $(PREFIX)gcc 2>/dev/null),)
  PREFIX := riscv-none-embed-
endif
ifeq ($(shell which $(PREFIX)gcc 2>/dev/null),)
  PREFIX := riscv64-unknown-elf-
endif

# Debian gcc-riscv64-unknown-elf ships picolibc (no nano.specs)
SPECS := --specs=nano.specs --specs=nosys.specs
ifeq ($(PREFIX),riscv64-unknown-elf-)
  SPECS := --specs=picolibc.specs
endif

CC       := $(PREFIX)gcc
AS       := $(PREFIX)gcc
OBJCOPY  := $(PREFIX)objcopy
OBJDUMP  := $(PREFIX)objdump
SIZE     := $(PREFIX)size

MCUFLAGS := -march=rv32imac_zicsr -mabi=ilp32 -msmall-data-limit=8
MCUFLAGS += -msave-restore -fmessage-length=0 -fsigned-char
MCUFLAGS += -ffunction-sections -fdata-sections -fno-common

DEFS     := -DCH32V20x_D6

INCLUDES := \
  -IUser \
  -IPlatform \
  -ISensors/lsm6dsv \
  -IMiddleware/vqf-c

CFLAGS   := $(MCUFLAGS) $(DEFS) $(INCLUDES) -Os -g3 -Wall -Wextra
CFLAGS   += -fno-math-errno
CFLAGS   += -Wno-unused-parameter
ASFLAGS  := $(MCUFLAGS) $(DEFS) -g3
CFLAGS   += $(SPECS)
ASFLAGS  += $(SPECS)
LDFLAGS  := $(MCUFLAGS) -T Startup/link.ld -nostartfiles
LDFLAGS  += -Wl,--gc-sections -Wl,-Map,$(BUILD)/$(TARGET).map
LDFLAGS  += $(SPECS) -lm

C_SRCS := \
  User/main.c \
  User/system_ch32v20x.c \
  User/ch32v20x_it.c \
  Platform/platform_ch32v203.c \
  Sensors/lsm6dsv/lsm6dsv.c \
  Middleware/vqf-c/vqf.c

AS_SRCS := Startup/startup_ch32v20x_D6.S

OBJS := $(addprefix $(BUILD)/,$(C_SRCS:.c=.o) $(AS_SRCS:.S=.o))

.PHONY: all clean flash size tree verify-zw

all: $(BUILD)/$(TARGET).elf $(BUILD)/$(TARGET).hex $(BUILD)/$(TARGET).bin size

$(BUILD)/$(TARGET).elf: $(OBJS)
	@mkdir -p $(dir $@)
	$(CC) $(OBJS) $(LDFLAGS) -o $@

$(BUILD)/$(TARGET).hex: $(BUILD)/$(TARGET).elf
	$(OBJCOPY) -O ihex $< $@

$(BUILD)/$(TARGET).bin: $(BUILD)/$(TARGET).elf
	$(OBJCOPY) -O binary $< $@

$(BUILD)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD)/%.o: %.S
	@mkdir -p $(dir $@)
	$(AS) $(ASFLAGS) -c $< -o $@

size: $(BUILD)/$(TARGET).elf
	$(SIZE) --format=berkeley $<

clean:
	rm -rf $(BUILD)

tree:
	@find . -type f -not -path './.git/*' -not -path './build/*' -not -path './vendor/mtkos-ch32v203-minimal/.git/*' | sort

# Example OpenOCD flash (WCH-Link); adjust scripts to your install
OPENOCD ?= openocd
flash: $(BUILD)/$(TARGET).elf
	$(OPENOCD) -f wch-riscv.cfg -c "program $(BUILD)/$(TARGET).elf verify reset exit"

# Verify every 1 kHz hot symbol + jal targets from updateGyr/updateAcc are < 0x8000
verify-zw: $(BUILD)/$(TARGET).elf
	@python3 scripts/verify_zw_hotpath.py $(BUILD)/$(TARGET).elf
