# CH32V203G6U6 + LSM6DSV + VQFC — GCC Makefile
# Toolchain: xPack / MRS riscv-none-elf-gcc or riscv-none-embed-gcc
#
#   make            # build build/firmware.elf
#   make flash      # optional: openocd + wch-link (adjust OPENOCD)
#
# MRS path: create a CH32V203 project, add User/Sensors/Middleware/Platform
# sources, use Startup/link.ld (32K/10K). SPL not required for this Makefile.

TARGET   ?= firmware
BUILD    ?= build

# Prefer newer xPack name, fall back to MRS embed toolchain
PREFIX   ?= riscv-none-elf-
ifeq ($(shell which $(PREFIX)gcc 2>/dev/null),)
  PREFIX := riscv-none-embed-
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
  -IMiddleware/vqfc

CFLAGS   := $(MCUFLAGS) $(DEFS) $(INCLUDES) -Os -g3 -Wall -Wextra
CFLAGS   += -Wno-unused-parameter
ASFLAGS  := $(MCUFLAGS) $(DEFS) -g3
LDFLAGS  := $(MCUFLAGS) -T Startup/link.ld -nostartfiles
LDFLAGS  += -Wl,--gc-sections -Wl,-Map,$(BUILD)/$(TARGET).map
LDFLAGS  += --specs=nano.specs --specs=nosys.specs -lm

C_SRCS := \
  User/main.c \
  User/system_ch32v20x.c \
  User/ch32v20x_it.c \
  Platform/platform_ch32v203.c \
  Sensors/lsm6dsv/lsm6dsv.c \
  Middleware/vqfc/vqfc.c

AS_SRCS := Startup/startup_ch32v20x_D6.S

OBJS := $(addprefix $(BUILD)/,$(C_SRCS:.c=.o) $(AS_SRCS:.S=.o))

.PHONY: all clean flash size tree

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
