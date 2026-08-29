# Cross toolchain for cross-compiling libmicroros.a (rcl/rclc + Micro-XRCE-DDS-Client)
# for testSTM (STM32H743XIH6, Cortex-M7, hard-float, bare metal - no HAL/CubeMX, see
# testSTM/README.md). Passed to `ros2 run micro_ros_setup build_firmware.sh`.
#
# MCU_FLAGS below MUST match testSTM/Makefile's MCU_FLAGS exactly - this produces a
# static lib of object code, and mismatched -mcpu/-mfpu/-mfloat-abi/-mthumb between it
# and the rest of the firmware is an ABI mismatch (float calling convention, instruction
# set) that link fine but corrupt data/crash at runtime, not a build-time error.

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER (set below) makes find_program search the host
# PATH as usual, so this resolves the same arm-none-eabi-gcc testSTM/Makefile uses.
find_program(ARM_GCC_PATH arm-none-eabi-gcc REQUIRED)
get_filename_component(ARM_TOOLCHAIN_BIN_DIR ${ARM_GCC_PATH} DIRECTORY)

set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_AR arm-none-eabi-ar)
set(CMAKE_RANLIB arm-none-eabi-ranlib)

# Cross-compiling bare metal: no OS/_start to link+run a try_compile test executable
# against, so restrict try_compile to producing a static library instead.
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(MCU_FLAGS "-mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard")

# Same newlib header workaround as testSTM/Makefile's NEWLIB_INCDIR: this toolchain's
# apt/.deb packaging splits newlib's string.h/stdint.h/etc into include/newlib/ instead of
# gcc's default search path ("string.h: No such file or directory" otherwise even though
# the toolchain is installed correctly). Resolved relative to the gcc actually found above,
# not hardcoded, so this still works if the toolchain lives somewhere else on another machine.
set(NEWLIB_INCDIR "${ARM_TOOLCHAIN_BIN_DIR}/../include/newlib")

# _POSIX_C_SOURCE: our newlib's time.h hides clock_gettime()/CLOCK_MONOTONIC (which
# rcutils' time_unix.c needs) behind __POSIX_VISIBLE, which -std=c11's __STRICT_ANSI__
# collapses to 0 unless a POSIX feature-test macro says otherwise. Only affects
# declarations seen while building this lib - the actual clock_gettime() symbol still has
# to be implemented in testSTM itself (arm-none-eabi's nosys.specs doesn't provide one),
# see app/src/microros_time.c.
# _POSIX_TIMERS/_POSIX_MONOTONIC_CLOCK: newlib's time.h guards clock_gettime()'s
# prototype and CLOCK_MONOTONIC's definition behind these SEPARATELY from
# _POSIX_C_SOURCE/__POSIX_VISIBLE (they're newlib's way of saying "this target actually
# implements POSIX timers", which nothing predefines for a bare arm-none-eabi target).
# We're asserting that's true because testSTM implements clock_gettime() itself, see
# app/src/microros_time.c.
set(CMAKE_C_FLAGS "${MCU_FLAGS} -DSTM32H743xx -D_POSIX_C_SOURCE=200809L -D_POSIX_TIMERS -D_POSIX_MONOTONIC_CLOCK -Os -g3 -ffunction-sections -fdata-sections -isystem ${NEWLIB_INCDIR}" CACHE STRING "" FORCE)

# rosidl_typesupport_c (uros fork, see mcu_ws/uros/rosidl_typesupport/rosidl_typesupport_c)
# unconditionally generates one tiny <msg>__type_support.cpp dispatcher per message, even
# for a pure-C build - so a working C++ header set is unavoidable even though testSTM
# itself is all C. Our toolchain package split these out into a separate .deb
# (libstdc++-arm-none-eabi-dev) that ~/tools/arm-none-eabi-toolchain didn't originally
# have; downloaded with `apt-get download` (no root) and merged into that same prefix
# under newlib/c++/ to match the existing NEWLIB_INCDIR split above.
set(CXX_INCDIR "${NEWLIB_INCDIR}/c++/13.2.1")
set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -isystem ${CXX_INCDIR} -isystem ${CXX_INCDIR}/arm-none-eabi -fno-exceptions -fno-rtti" CACHE STRING "" FORCE)
set(CMAKE_EXE_LINKER_FLAGS "${MCU_FLAGS} --specs=nano.specs --specs=nosys.specs" CACHE STRING "" FORCE)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
