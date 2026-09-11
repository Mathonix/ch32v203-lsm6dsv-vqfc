# Host (x86_64/aarch64) float vs fixed VQF replay
CC ?= gcc
ROOT := ../..
CFLAGS := -O2 -g -Wall -Wextra -std=c11 -DVQF_FIXED_HOST=1
CFLAGS += -I. -I$(ROOT)/Platform -I$(ROOT)/Middleware/vqf_fixed -I$(ROOT)/Middleware/vqf-c
CFLAGS += -D_GNU_SOURCE
LDFLAGS := -lm

FIXED_SRCS := \
  $(ROOT)/Middleware/vqf_fixed/vqf_fixed.c \
  $(ROOT)/Middleware/vqf_fixed/vqf_fixed_math.c \
  $(ROOT)/Middleware/vqf_fixed/vqf_fixed_quat.c \
  $(ROOT)/Middleware/vqf_fixed/vqf_fixed_biquad.c \
  $(ROOT)/Middleware/vqf_fixed/vqf_fixed_ldlt.c

FLOAT_SRCS := $(ROOT)/Middleware/vqf-c/vqf.c

SRCS := host_main.c $(FIXED_SRCS) $(FLOAT_SRCS)
OBJS := $(SRCS:.c=.o)
# flatten object names
BUILD := build
BIN := $(BUILD)/vqf_fixed_replay

.PHONY: all clean run

all: $(BIN)

$(BUILD):
	mkdir -p $(BUILD)

$(BIN): host_main.c $(FIXED_SRCS) $(FLOAT_SRCS) | $(BUILD)
	$(CC) $(CFLAGS) -o $@ host_main.c $(FIXED_SRCS) $(FLOAT_SRCS) $(LDFLAGS)

run: $(BIN)
	./$(BIN) --seconds 10

clean:
	rm -rf $(BUILD)
