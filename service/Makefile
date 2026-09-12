# Out-of-tree RT-SMART userapp build for the k230-lr2021-radio service.
#
# Expects (see scripts/build.sh):
#   LILYGO_SDK — path to the T-Display-K230_canmv_rt checkout
#   MPP_SRC_DIR / RTSMART_SRC_DIR — set by the SDK's build_env.sh
#   RTT_EXEC_PATH — the riscv64-unknown-linux-musl toolchain bin/
#
# Usage (from this directory):
#   source $(LILYGO_SDK)/canmv_k230/src/rtsmart/mpp/build_env.sh
#   make
#
# Output: ../build/lora_radio.elf

# Capture our own path BEFORE including the SDK mk files (they append to
# MAKEFILE_LIST).
APP_DIR := $(patsubst %/,%,$(dir $(abspath $(firstword $(MAKEFILE_LIST)))))
OUT_DIR := $(APP_DIR)/../build

-include $(MPP_SRC_DIR)/userapps/sample/mpp.mk
-include $(MPP_SRC_DIR)/userapps/sample/rt-smart.mk

APP_CPPS := $(APP_DIR)/src/main.cpp \
            $(APP_DIR)/src/radio_worker.cpp \
            $(APP_DIR)/src/serial_server.cpp \
            $(wildcard $(APP_DIR)/../vendor/lora_lib/*.cpp) \
            $(wildcard $(APP_DIR)/../vendor/lora_lib/modules/*/*.cpp) \
            $(wildcard $(APP_DIR)/../vendor/lora_lib/protocols/PhysicalLayer/*.cpp) \
            $(wildcard $(APP_DIR)/../vendor/lora_lib/utils/*.cpp) \
            $(APP_DIR)/../vendor/fpioa/rt_fpioa.c
APP_SRCS := $(APP_CPPS)
APP_OBJS := $(patsubst %.cpp,%.o,$(notdir $(filter %.cpp,$(APP_SRCS)))) \
            $(patsubst %.c,%.o,$(notdir $(filter %.c,$(APP_SRCS))))
VPATH := $(sort $(dir $(APP_SRCS)))

# RT-SMART kernel/userspace headers (cconfig.h, rtdevice.h, ...) — these are
# the -I set from the kernel's rt-smart.mk, which the sample mk files don't
# export.
KERNEL_INCLUDES := -I$(MPP_SRC_DIR)/kernel/include \
	-I$(RTSMART_SRC_DIR)/kernel/rt-thread/include \
	-I$(RTSMART_SRC_DIR)/kernel/rt-thread/components/drivers/include \
	-I$(RTSMART_SRC_DIR)/kernel/rt-thread/components/libc/compilers/musl \
	-I$(RTSMART_SRC_DIR)/kernel/rt-thread/components/lwp \
	-I$(RTSMART_SRC_DIR)/kernel/rt-thread/components/lwp/arch/risc-v/rv64 \
	-I$(RTSMART_SRC_DIR)/kernel/bsp/maix3 \
	-I$(RTSMART_SRC_DIR)/kernel/bsp/maix3/board \
	-I$(RTSMART_SRC_DIR)/kernel/bsp/maix3/drivers/interdrv/gpio \
	-I$(RTSMART_SRC_DIR)/kernel/bsp/maix3/c908

CPPFLAGS += \
  -I$(APP_DIR)/src \
  -I$(APP_DIR)/../vendor/lora_lib \
  -I$(APP_DIR)/../vendor/lora_lib/modules/LR20xx \
  -I$(APP_DIR)/../vendor/lora_lib/modules/SX126x \
  -I$(APP_DIR)/../vendor/lora_lib/modules/SX127x \
  -I$(APP_DIR)/../vendor/lora_lib/protocols/PhysicalLayer \
  -I$(APP_DIR)/../vendor/lora_lib/utils \
  -I$(APP_DIR)/../vendor/fpioa \
  $(KERNEL_INCLUDES) \
  $(MPP_USER_CFLGAS)

# -Werror comes in from rt-smart.mk; the vendored (upstream) RadioLib code
# isn't clean under it, so relax for the whole app — our src/ stays clean.
CPPFLAGS += -Wno-unused-variable -Wno-unused-function -Wno-sign-compare -Wno-error

LDFLAGS += -L$(MPP_LIB_PATH) -lstdc++

all: $(OUT_DIR)/lora_radio.elf

$(OUT_DIR)/lora_radio.elf: $(APP_OBJS)
	@mkdir -p $(OUT_DIR)
	$(CPP) $(CC_CFLAGS) $(LINKFLAG) -o $@ $^ $(MPP_LIBS) $(LDFLAGS)

%.o: %.cpp
	$(CPP) $(CC_CFLAGS) $(CPPFLAGS) -c $< -o $@

%.o: %.c
	$(CC) $(CC_CFLAGS) $(CPPFLAGS) -c $< -o $@

clean:
	rm -rf $(OUT_DIR) $(APP_OBJS)

.PHONY: all clean
