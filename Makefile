# CH32V203G6U6 + LSM6DSV + multi-algo fusion — GCC Makefile
# Toolchain: xPack / MRS riscv-none-elf-gcc or riscv-none-embed-gcc
#
#   make            # build build/firmware.elf
#   make flash      # optional: openocd + wch-link (adjust OPENOCD)
#   make verify-zw  # VQF-fxp hot-path symbols still in R0WAIT
#
# Flash: G6U6 R0WAIT=32KB zero-wait + ~188KB NZW code + 4KB algo_cfg @ 0x37000
# ALGO_RAM 3KB @ 0x20000000 for Mahony/Comp SRAM execute; RAM 7KB remainder.

TARGET   ?= firmware
BUILD    ?= build

# Prefer picolibc (riscv64-unknown-elf) so libm lands in .text_zw via link.ld wildcards.
PREFIX   ?= riscv64-unknown-elf-
ifeq ($(shell which $(PREFIX)gcc 2>/dev/null),)
  PREFIX := riscv-none-elf-
endif
ifeq ($(shell which $(PREFIX)gcc 2>/dev/null),)
  PREFIX := riscv-none-embed-
endif

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
  -IMiddleware/vqf-c \
  -IMiddleware/vqf-fxp \
  -IMiddleware/fusion

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
  Platform/algo_cfg.c \
  Sensors/lsm6dsv/lsm6dsv.c \
  Middleware/vqf-fxp/vqf_fxp.c \
  Middleware/fusion/fusion_vqf.c \
  Middleware/fusion/fusion_select.c \
  Middleware/fusion/mahony.c \
  Middleware/fusion/complementary.c

AS_SRCS := Startup/startup_ch32v20x_D6.S

OBJS := $(addprefix $(BUILD)/,$(C_SRCS:.c=.o) $(AS_SRCS:.S=.o))

.PHONY: all clean flash size tree verify-zw

all: $(BUILD)/$(TARGET).elf $(BUILD)/$(TARGET).hex $(BUILD)/$(TARGET).bin size

$(BUILD)/$(TARGET).elf: $(OBJS) Startup/link.ld
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

OPENOCD ?= openocd
flash: $(BUILD)/$(TARGET).elf
	$(OPENOCD) -f wch-riscv.cfg -c "program $(BUILD)/$(TARGET).elf verify reset exit"

verify-zw: $(BUILD)/$(TARGET).elf
	@python3 scripts/verify_zw_hotpath.py $(BUILD)/$(TARGET).elf
