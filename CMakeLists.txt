cmake_minimum_required(VERSION 3.13)

# Pico SDK
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(dishwasher-pico C CXX)

pico_sdk_init()

add_executable(dishwasher
        src/main.c
        src/ir/cmd_listener.c
        include/ir/cmd_listener.h
        src/ir/gpio_edge_receiver.c
        src/ir/gpio_edge_receiver.h
        src/ir/pulse_detector.c
        src/ir/pulse_detector.h
        src/ir/nec_decoder.c
        src/ir/nec_decoder.h
)

target_include_directories(dishwasher PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/src
        ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(dishwasher pico_stdlib)

# .uf2, .bin, .hex
pico_add_extra_outputs(dishwasher)
