# Board firmware: core-only hash runner (display layer lands with the kit).
# SDK glue is included from the SDK's own external/ directory; no SDK
# file is copied or edited. Sim code is integer-only library calls,
# so the same sources produce identical states on host/RP2040/RP2350.
#
# Platform selection (see board/README.md):
#   -DTOY_PICO_PLATFORM=rp2040   (default; the historical build:
#                                 SDK platform rp2040, board pico)
#   -DTOY_PICO_PLATFORM=rp2350   (Cortex-M33: SDK platform rp2350,
#                                 which it expands to rp2350-arm-s;
#                                 default board pico2)
# Always use a separate build directory per platform (the SDK
# refuses a platform change inside one directory anyway).
# Passthrough, for SDK muscle memory: an explicit
# -DPICO_PLATFORM / $PICO_PLATFORM (with no TOY_PICO_PLATFORM)
# selects the same way, and a lone -DPICO_BOARD is left for the
# SDK to derive the platform from; TOY_PICO_PLATFORM is then set
# to the platform actually selected. If several knobs are given
# they must agree. rp2350-riscv (and host) are deliberately
# rejected: TOY-0 targets the Cortex-M33 pair, and the RISC-V
# cores/toolchain are unaudited.
cmake_minimum_required(VERSION 3.13)

if(NOT DEFINED PICO_SDK_PATH AND DEFINED ENV{PICO_SDK_PATH})
    set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})
endif()
if(NOT DEFINED PICO_SDK_PATH)
    message(FATAL_ERROR "PICO_SDK_PATH not set (env or -D)")
endif()

# Which knobs did the user actually supply? (Test TOY_ first:
# a -D lands in the cache before this file runs, and so does our
# own value from a previous configure of the same directory -
# both count as the established selection for that directory.)
set(_toy_knob_given 0)
if(DEFINED TOY_PICO_PLATFORM)
    set(_toy_knob_given 1)
endif()
# On a reconfigure the PICO_PLATFORM cache holds our own derived
# value (the SDK expands rp2350 to rp2350-arm-s), which counts as
# agreement; a genuine mismatch can only mean a fresh -D or a
# changed knob, both of which want a fresh build directory.
set(_toy_given_pico_platform "")
if(DEFINED PICO_PLATFORM AND NOT PICO_PLATFORM STREQUAL "")
    set(_toy_given_pico_platform "${PICO_PLATFORM}")
elseif(DEFINED ENV{PICO_PLATFORM})
    set(_toy_given_pico_platform "$ENV{PICO_PLATFORM}")
endif()
set(_toy_board_given 0)
if((DEFINED PICO_BOARD AND NOT PICO_BOARD STREQUAL "")
   OR DEFINED ENV{PICO_BOARD})
    set(_toy_board_given 1)
endif()

set(_toy_platform "")
if(_toy_knob_given)
    string(TOLOWER "${TOY_PICO_PLATFORM}" _toy_platform)
    if(NOT _toy_platform STREQUAL "rp2040"
       AND NOT _toy_platform STREQUAL "rp2350")
        message(FATAL_ERROR
            "TOY_PICO_PLATFORM must be rp2040 or rp2350 "
            "(got '${TOY_PICO_PLATFORM}'); rp2350-riscv is out of "
            "scope (Cortex-M33 only, see comment above)")
    endif()
    if(NOT _toy_given_pico_platform STREQUAL "")
        if(_toy_platform STREQUAL "rp2040" AND
           NOT _toy_given_pico_platform STREQUAL "rp2040")
            message(FATAL_ERROR
                "PICO_PLATFORM='${_toy_given_pico_platform}' disagrees "
                "with TOY_PICO_PLATFORM=rp2040; align them "
                "(fresh build dir per platform)")
        endif()
        if(_toy_platform STREQUAL "rp2350" AND
           NOT _toy_given_pico_platform STREQUAL "rp2350" AND
           NOT _toy_given_pico_platform STREQUAL "rp2350-arm-s")
            message(FATAL_ERROR
                "PICO_PLATFORM='${_toy_given_pico_platform}' disagrees "
                "with TOY_PICO_PLATFORM=rp2350; align them "
                "(fresh build dir per platform)")
        endif()
    endif()
elseif(NOT _toy_given_pico_platform STREQUAL "")
    # PICO_PLATFORM passthrough: derive the toy knob from it.
    if(_toy_given_pico_platform STREQUAL "rp2040")
        set(_toy_platform rp2040)
    elseif(_toy_given_pico_platform STREQUAL "rp2350"
           OR _toy_given_pico_platform STREQUAL "rp2350-arm-s")
        set(_toy_platform rp2350)
    else()
        message(FATAL_ERROR
            "PICO_PLATFORM='${_toy_given_pico_platform}' is not a "
            "supported toy platform (rp2040 or rp2350/rp2350-arm-s); "
            "rp2350-riscv is out of scope (Cortex-M33 only)")
    endif()
elseif(_toy_board_given)
    # Lone PICO_BOARD: let the SDK derive the platform from the
    # board header; _toy_platform is filled in after sdk init.
    set(_toy_platform "")
else()
    set(_toy_platform rp2040)
endif()

# The RP2350 hardware FPU/DCP must never run cartridge maths; a
# hard-float ABI would route float/double through it. Refuse it
# unless the selection is explicitly RP2040 (where the SDK
# ignores it with a warning). Implementation pins are applied
# on the target below.
if(PICO_HARD_FLOAT_ABI AND NOT _toy_platform STREQUAL "rp2040")
    message(FATAL_ERROR
        "PICO_HARD_FLOAT_ABI is not allowed unless RP2040 is "
        "explicitly selected: on RP2350 the sim must stay "
        "muse-softfloat integer code (TOY-0 s2)")
endif()

if(_toy_platform STREQUAL "rp2350")
    set(TOY_PICO_PLATFORM rp2350 CACHE STRING
        "Toy board platform: rp2040 or rp2350 (fresh build dir per platform)"
        FORCE)
    set(PICO_PLATFORM rp2350 CACHE STRING "PICO Build platform" FORCE)
    if(NOT _toy_board_given)
        set(PICO_BOARD pico2 CACHE STRING "PICO target board" FORCE)
    endif()
elseif(_toy_platform STREQUAL "rp2040")
    set(TOY_PICO_PLATFORM rp2040 CACHE STRING
        "Toy board platform: rp2040 or rp2350 (fresh build dir per platform)"
        FORCE)
    set(PICO_PLATFORM rp2040 CACHE STRING "PICO Build platform" FORCE)
    if(NOT _toy_board_given)
        set(PICO_BOARD pico CACHE STRING "PICO target board" FORCE)
    endif()
endif()
if(DEFINED TOY_PICO_PLATFORM)
    set_property(CACHE TOY_PICO_PLATFORM PROPERTY STRINGS rp2040 rp2350)
endif()

include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(toy_board C CXX ASM)
pico_sdk_init()

# The platform actually in force, after SDK derivation/expansion
# (this is also where a lone-PICO_BOARD selection lands).
if(PICO_PLATFORM STREQUAL "rp2040")
    set(_toy_platform_actual rp2040)
elseif(PICO_PLATFORM STREQUAL "rp2350"
       OR PICO_PLATFORM STREQUAL "rp2350-arm-s")
    set(_toy_platform_actual rp2350)
else()
    message(FATAL_ERROR
        "toy_board supports SDK platforms rp2040 and "
        "rp2350/rp2350-arm-s only (got '${PICO_PLATFORM}'); "
        "rp2350-riscv is out of scope (Cortex-M33 only)")
endif()
if(NOT _toy_platform STREQUAL ""
   AND NOT _toy_platform STREQUAL _toy_platform_actual)
    message(FATAL_ERROR
        "selected toy platform ${_toy_platform} but the SDK "
        "configured ${_toy_platform_actual} (PICO_PLATFORM="
        "${PICO_PLATFORM}, PICO_BOARD=${PICO_BOARD}); align the "
        "knobs (fresh build dir per platform)")
endif()
set(TOY_PICO_PLATFORM ${_toy_platform_actual} CACHE STRING
    "Toy board platform: rp2040 or rp2350 (fresh build dir per platform)"
    FORCE)
set_property(CACHE TOY_PICO_PLATFORM PROPERTY STRINGS rp2040 rp2350)
message(STATUS
    "toy_board: TOY_PICO_PLATFORM=${_toy_platform_actual} "
    "PICO_PLATFORM=${PICO_PLATFORM} PICO_BOARD=${PICO_BOARD}")

execute_process(COMMAND git -C ${CMAKE_CURRENT_LIST_DIR} rev-parse --short HEAD
    OUTPUT_VARIABLE TOY_GIT_HEAD OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET)
if(NOT TOY_GIT_HEAD)
    set(TOY_GIT_HEAD "unknown")
endif()

set(TOY_ROOT ${CMAKE_CURRENT_LIST_DIR}/..)
set(LIB ${TOY_ROOT}/libs/muse-softfloat)
file(GLOB LIB_SRC ${LIB}/src/*.c)

# Internal-flash checkpoint backend selftest (board/ckpt_flash.h):
# OFF by default - it only compiles an uncalled one-shot helper
# for the first on-device write/read proof. An always-on
# boot-time flash write would burn erase cycles and could
# destroy a real resume slot if ever shipped by accident.
option(TOY_CKPT_FLASH_SELFTEST
    "Compile the (uncalled) flash checkpoint round-trip selftest" OFF)

add_executable(toy_board
    main_board.c
    clock_board.c
    ckpt_flash.c
    ${TOY_ROOT}/core/toy_ckpt.c
    ${TOY_ROOT}/core/toy_fb.c
    ${TOY_ROOT}/core/toy_carts.c
    ${TOY_ROOT}/core/toy_input.c
    ${TOY_ROOT}/core/toy_shell.c
    ${TOY_ROOT}/core/cart_fig8.c
    ${TOY_ROOT}/core/cart_pendulum.c
    ${TOY_ROOT}/core/cart_carousel.c
    ${TOY_ROOT}/core/cart_stagnation.c
    ${TOY_ROOT}/core/cart_microscope.c
    ${TOY_ROOT}/core/cart_harmonograph.c
    ${TOY_ROOT}/core/cart_lorenz.c
    ${TOY_ROOT}/core/cart_intervallens.c
    ${TOY_ROOT}/core/cart_pendwave.c
    ${TOY_ROOT}/core/cart_orrery.c
    ${TOY_ROOT}/core/cart_cloud.c
    ${TOY_ROOT}/core/cart_mandelbrot.c
    ${TOY_ROOT}/core/toy_fft.c
    ${TOY_ROOT}/core/toy_signal.c
    ${TOY_ROOT}/core/cart_orbitspec.c
    ${TOY_ROOT}/core/cart_julia.c
    ${TOY_ROOT}/core/cart_gravity.c
    ${TOY_ROOT}/core/cart_gpend.c
    ${TOY_ROOT}/core/cart_seismo.c
    ${TOY_ROOT}/core/cart_attitude.c
    ${TOY_ROOT}/core/cart_synspec.c
    ${TOY_ROOT}/core/cart_heartbeat.c
    ${TOY_ROOT}/core/cart_solclock.c
    ${TOY_ROOT}/core/cart_pond.c
    ${TOY_ROOT}/core/cart_shepherd.c
    ${LIB_SRC}
)
target_include_directories(toy_board PRIVATE ${LIB}/include ${TOY_ROOT}/core)
target_compile_definitions(toy_board PRIVATE TOY_GIT_HEAD="${TOY_GIT_HEAD}")
# Status LED GPIO; -1 = no plain LED (Waveshare RP2350-Touch-
# AMOLED-1.75: GPIO25 is UART1 RX). Default: board header pin.
set(TOY_BOARD_LED_PIN "" CACHE STRING "Status LED GPIO, -1 for none")
if(NOT TOY_BOARD_LED_PIN STREQUAL "")
    target_compile_definitions(toy_board PRIVATE
        TOY_BOARD_LED_PIN=${TOY_BOARD_LED_PIN})
endif()
target_compile_options(toy_board PRIVATE -O2)
# pico_flash provides flash_safe_execute, hardware_flash the
# erase/program primitives it wraps (board/ckpt_flash.c).
target_link_libraries(toy_board pico_stdlib pico_unique_id pico_flash hardware_flash)
# This firmware never launches core 1, so flash_safe_execute
# may assume it safe and use the IRQ-only path (without this,
# pico_flash refuses as NOT_PERMITTED when pico_multicore is
# linked but no lockout victim runs on core 1). A future
# core-1 firmware must drop this and call
# flash_safe_execute_core_init() on core 1 instead.
target_compile_definitions(toy_board PRIVATE PICO_FLASH_ASSUME_CORE1_SAFE=1)
if(TOY_CKPT_FLASH_SELFTEST)
    target_compile_definitions(toy_board PRIVATE TOY_CKPT_FLASH_SELFTEST=1)
endif()
# The checkpoint backend is deliberately uncalled by this proof
# firmware; root its entry points so --gc-sections keeps the
# linked API in the UF2 for Friday's wiring to call.
target_link_options(toy_board PRIVATE
    -Wl,--undefined=toy_ckpt_flash_layout
    -Wl,--undefined=toy_ckpt_flash_last_safe_rc
    -Wl,--undefined=toy_ckpt_flash_slot_xip
    -Wl,--undefined=toy_ckpt_flash_read_slot
    -Wl,--undefined=toy_ckpt_flash_find_newest
    -Wl,--undefined=toy_ckpt_flash_load_newest
    -Wl,--undefined=toy_ckpt_flash_write_slot
    -Wl,--undefined=toy_ckpt_flash_write_next
    -Wl,--undefined=toy_ckpt_flash_erase_slot
    -Wl,--undefined=toy_ckpt_flash_erase_all)
if(TOY_CKPT_FLASH_SELFTEST)
    target_link_options(toy_board PRIVATE
        -Wl,--undefined=toy_ckpt_flash_selftest_roundtrip)
endif()
if(_toy_platform_actual STREQUAL "rp2350")
    # Float/double discipline on RP2350 (audit: board/README.md).
    # The SDK defaults would silently substitute hardware paths:
    # pico_float=pico_vfp (Cortex-M33 SP FPU, -mfloat-abi=softfp)
    # and pico_double=pico_dcp (the RP2350 double coprocessor).
    # The sim (reset/step/apply/feed, all of libs/muse-softfloat)
    # calls only msf_* integer code; the only native-float users
    # in this binary are the cartridges' display-only render()
    # double scaling and the SDK printf %f formatter. Pin both
    # SDK switches off the hardware, per target (these override
    # any PICO_DEFAULT_FLOAT_IMPL/_DOUBLE_IMPL for toy_board):
    #   float=none     - no SP consumer exists in core/board;
    #                    fail closed (panic stubs), and its
    #                    -msoft-float stops the compiler emitting
    #                    VFP instructions anywhere in this target.
    #   double=compiler - render()/printf keep working through
    #                    libgcc software routines, as on RP2040
    #                    (whose ROM routines are already software,
    #                    so the rp2040 build is left untouched).
    pico_set_float_implementation(toy_board none)
    pico_set_double_implementation(toy_board compiler)
endif()
pico_enable_stdio_usb(toy_board 1)
pico_enable_stdio_uart(toy_board 0)
pico_add_extra_outputs(toy_board)

# --- toy_sensor_test (toy5-sensors parcel; RP2350 only) ----------
# Standalone I2C sensor bring-up firmware (board/SENSORS.md):
# QMI8658C IMU + CST9217 touch + PCF85063A RTC on the Waveshare
# RP2350-Touch-AMOLED-1.75. No cartridges, no display. Appended
# after toy_board so sibling parcels with their own appended
# targets union-merge trivially. RP2040 configures exactly as
# before (this whole block is skipped).
if(_toy_platform_actual STREQUAL "rp2350")
    add_executable(toy_sensor_test
        main_sensor_test.c
        imu_qmi8658.c
        touch_cst9217.c
        rtc_pcf85063.c
    )
    target_include_directories(toy_sensor_test PRIVATE
        ${CMAKE_CURRENT_LIST_DIR})
    target_compile_definitions(toy_sensor_test PRIVATE
        TOY_GIT_HEAD="${TOY_GIT_HEAD}")
    if(NOT TOY_BOARD_LED_PIN STREQUAL "")
        target_compile_definitions(toy_sensor_test PRIVATE
            TOY_BOARD_LED_PIN=${TOY_BOARD_LED_PIN})
    endif()
    target_compile_options(toy_sensor_test PRIVATE
        -O2 -Wall -Wextra -Werror)
    target_link_libraries(toy_sensor_test
        pico_stdlib pico_unique_id hardware_i2c)
    # Same no-hardware-float pins as toy_board (see above): the
    # drivers are integer-only, so nothing here wants the FPU
    # or DCP either; printf keeps libgcc software double.
    pico_set_float_implementation(toy_sensor_test none)
    pico_set_double_implementation(toy_sensor_test compiler)
    pico_enable_stdio_usb(toy_sensor_test 1)
    pico_enable_stdio_uart(toy_sensor_test 0)
    pico_add_extra_outputs(toy_sensor_test)
endif()
