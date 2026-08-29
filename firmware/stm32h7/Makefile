##
## Makefile toi gian - build thu startup + linker script cho STM32H743XIH6
## Yeu cau: arm-none-eabi-gcc toolchain da co trong PATH
##
## Cac lenh:
##   make            -> build ra build/firmware.elf/.bin/.hex + in size
##   make clean      -> xoa thu muc build
##

TARGET   = firmware
BUILD_DIR = build
HOST_BUILD_DIR = build-host
BUILD ?= release

PREFIX  = arm-none-eabi-
CC      = $(PREFIX)gcc
OBJCOPY = $(PREFIX)objcopy
SIZE    = $(PREFIX)size
HOST_CC ?= cc

# ---- MCU: STM32H743XIH6 (Cortex-M7, co FPU double-precision fpv5-d16) ----
MCU_FLAGS = -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard

# Workaround: ban toolchain arm-none-eabi-gcc lay tu apt/.deb (ca ban chinh
# thuc tu developer.arm.com deu co the gap, tuy cach dong goi) tach rieng
# header newlib (string.h, stdint.h chuan, ...) vao thu muc "newlib/" thay vi
# de trong duong tim mac dinh cua gcc - gay loi "string.h: No such file or
# directory" du toolchain cai dung. Tu tim thu muc do TUONG DOI theo vi tri
# thuc te cua arm-none-eabi-gcc trong PATH (khong hardcode duong dan tuyet
# doi) nen van dung duoc du toolchain cai o may khac/vi tri khac.
NEWLIB_INCDIR := $(wildcard $(dir $(shell command -v $(CC)))../include/newlib)

# libmicroros.a + headers - cay ra tu microros/ (xem microros/toolchain.cmake, KHONG
# build lai o day, chi link vao). _POSIX_C_SOURCE/_POSIX_TIMERS/_POSIX_MONOTONIC_CLOCK
# phai giong het cac define da dung khi build libmicroros.a, neu khong <time.h>
# (app/src/microros_time.c, va cac header rcl/rclc include no) se an clock_gettime()/
# CLOCK_MONOTONIC di (xem microros/toolchain.cmake).
MICROROS_DIR    = microros/firmware/build
MICROROS_INCDIR = $(MICROROS_DIR)/include
MICROROS_LIB    = $(MICROROS_DIR)/libmicroros.a

DEFINES = -DSTM32H743xx -D_POSIX_C_SOURCE=200809L -D_POSIX_TIMERS -D_POSIX_MONOTONIC_CLOCK

ifeq ($(BUILD),release)
OPT_FLAGS = -O2 -g1 -DNDEBUG
else ifeq ($(BUILD),debug)
OPT_FLAGS = -Og -g3
else
$(error BUILD must be release or debug)
endif

LIB_DIR     = lib
LIB_INCDIR  = $(LIB_DIR)/inc
LIB_SRCDIR  = $(LIB_DIR)/src

APP_DIR     = app
APP_INCDIR  = $(APP_DIR)/inc
APP_SRCDIR  = $(APP_DIR)/src

C_SOURCES = \
	startup_stm32h743xx.c \
	main.c \
	$(wildcard $(LIB_SRCDIR)/*.c) \
	$(wildcard $(APP_SRCDIR)/*.c)

LDSCRIPT = stm32h743xihx_flash.ld

CFLAGS  = $(MCU_FLAGS) $(DEFINES) -Wall -Wextra $(OPT_FLAGS) -std=c11 \
          -I$(LIB_INCDIR) -I$(APP_INCDIR) -I$(MICROROS_INCDIR) \
          $(if $(NEWLIB_INCDIR),-isystem $(NEWLIB_INCDIR)) \
          -ffunction-sections -fdata-sections \
          -fstack-usage -fno-common \
          -MMD -MP -MF"$(BUILD_DIR)/$(*F).d"

# -lstdc++ -lsupc++: libmicroros.a co vai file .cpp (rosidl_typesupport_c luon sinh 1
# ham dispatcher C++ nho cho moi message, xem microros/toolchain.cmake) - CC
# ($(PREFIX)gcc, driver C) khong tu link libstdc++ nhu $(PREFIX)g++ se lam, phai xin
# tay. nano.specs tu doi -lstdc++/-lsupc++ sang ban "_nano" gon hon.
LDFLAGS = $(MCU_FLAGS) -T$(LDSCRIPT) \
          -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref \
          -Wl,--gc-sections \
          --specs=nano.specs --specs=nosys.specs \
          $(MICROROS_LIB) -lstdc++ -lsupc++ -lm

OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))

all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin
	$(SIZE) $(BUILD_DIR)/$(TARGET).elf

test: $(HOST_BUILD_DIR)/test_baby_alpha2_protocol
	$<

$(HOST_BUILD_DIR)/test_baby_alpha2_protocol: test/test_baby_alpha2_protocol.c \
		$(APP_SRCDIR)/baby_alpha2_protocol.c $(APP_INCDIR)/baby_alpha2_protocol.h | $(HOST_BUILD_DIR)
	$(HOST_CC) -std=c11 -Wall -Wextra -Werror -O2 \
		-I$(LIB_INCDIR) -I$(APP_INCDIR) $(filter %.c,$^) -lm -o $@

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) $(LDSCRIPT) | $(BUILD_DIR)
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(OBJCOPY) -O ihex $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(OBJCOPY) -O binary -S $< $@

$(BUILD_DIR):
	mkdir -p $@

$(HOST_BUILD_DIR):
	mkdir -p $@

flash: $(BUILD_DIR)/$(TARGET).bin
	st-flash --reset write $(BUILD_DIR)/$(TARGET).bin 0x08000000

clean:
	rm -rf $(BUILD_DIR) $(HOST_BUILD_DIR)

.PHONY: all clean flash test

-include $(wildcard $(BUILD_DIR)/*.d)
