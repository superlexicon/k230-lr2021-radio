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

-include $(MPP_SRC_DIR)/userapps/sample/mpp.mk
-include $(MPP_SRC_DIR)/userapps/sample/rt-smart.mk

APP_DIR := $(patsubst %/,%,$(dir $(abspath $(lastword $(MAKEFILE_LIST)))))
OUT_DIR := $(APP_DIR)/../build

# Our sources + the vendored RadioLib fork (mirrors the LILYGO tree's
# recursive-glob: every lora_lib .cpp compiles into the same ELF).
APP_CPPS := $(APP_DIR)/src/main.cpp \
            $(APP_DIR)/src/radio_worker.cpp \
            $(APP_DIR)/src/serial_server.cpp \
            $(wildcard $(APP_DIR)/../vendor/lora_lib/*.cpp) \
            $(wildcard $(APP_DIR)/../vendor/lora_lib/modules/*/*.cpp) \
            $(wildcard $(APP_DIR)/../vendor/lora_lib/protocols/PhysicalLayer/*.cpp) \
            $(wildcard $(APP_DIR)/../vendor/lora_lib/utils/*.cpp)
APP_OBJS := $(patsubst %.cpp,%.o,$(notdir $(APP_CPPS)))
VPATH := $(sort $(dir $(APP_CPPS)))

CPPFLAGS += \
  -I$(APP_DIR)/src \
  -I$(APP_DIR)/../vendor/lora_lib \
  -I$(APP_DIR)/../vendor/lora_lib/modules/LR20xx \
  -I$(APP_DIR)/../vendor/lora_lib/modules/SX126x \
  -I$(APP_DIR)/../vendor/lora_lib/modules/SX127x \
  -I$(APP_DIR)/../vendor/lora_lib/protocols/PhysicalLayer \
  -I$(APP_DIR)/../vendor/lora_lib/utils \
  -I$(APP_DIR)/../vendor/fpioa

# RadioLib needs Arduino-ish shims the k230Hal provides; keep warnings sane
# for the vendored (upstream) code — ours builds with -Werror.
CPPFLAGS += -Wno-unused-variable -Wno-unused-function -Wno-sign-compare

LDFLAGS += -lstdc++

all: $(OUT_DIR)/lora_radio.elf

$(OUT_DIR)/lora_radio.elf: $(APP_OBJS)
	@mkdir -p $(OUT_DIR)
	$(LINK) $(LINKFLAG) -o $@ $^ $(MPP_LIBS) $(LDFLAGS)

%.o: %.cpp
	$(CPP) $(CPPFLAGS) $(CC_CFLAGS) -c $< -o $@

clean:
	rm -rf $(OUT_DIR) $(APP_OBJS)

.PHONY: all clean
