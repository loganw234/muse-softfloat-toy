# Board firmware: core-only hash runner (display layer lands with the kit).
# SDK glue is included from the SDK's own external/ directory; no SDK
# file is copied or edited. Sim code is integer-only library calls,
# so the same sources produce identical states on host/RP2040/RP2350.
cmake_minimum_required(VERSION 3.13)

if(NOT DEFINED PICO_SDK_PATH AND DEFINED ENV{PICO_SDK_PATH})
    set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})
endif()
if(NOT DEFINED PICO_SDK_PATH)
    message(FATAL_ERROR "PICO_SDK_PATH not set (env or -D)")
endif()
include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(toy_board C CXX ASM)
pico_sdk_init()

execute_process(COMMAND git -C ${CMAKE_CURRENT_LIST_DIR} rev-parse --short HEAD
    OUTPUT_VARIABLE TOY_GIT_HEAD OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET)
if(NOT TOY_GIT_HEAD)
    set(TOY_GIT_HEAD "unknown")
endif()

set(TOY_ROOT ${CMAKE_CURRENT_LIST_DIR}/..)
set(LIB ${TOY_ROOT}/libs/muse-softfloat)
file(GLOB LIB_SRC ${LIB}/src/*.c)

add_executable(toy_board
    main_board.c
    ${TOY_ROOT}/core/toy_fb.c
    ${TOY_ROOT}/core/toy_carts.c
    ${TOY_ROOT}/core/cart_fig8.c
    ${TOY_ROOT}/core/cart_pendulum.c
    ${TOY_ROOT}/core/cart_pendwave.c
    ${LIB_SRC}
)
target_include_directories(toy_board PRIVATE ${LIB}/include ${TOY_ROOT}/core)
target_compile_definitions(toy_board PRIVATE TOY_GIT_HEAD="${TOY_GIT_HEAD}")
target_compile_options(toy_board PRIVATE -O2)
target_link_libraries(toy_board pico_stdlib pico_unique_id)
pico_enable_stdio_usb(toy_board 1)
pico_enable_stdio_uart(toy_board 0)
pico_add_extra_outputs(toy_board)
