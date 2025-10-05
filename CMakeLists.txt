cmake_minimum_required(VERSION 3.13)

# Pico SDK
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(dishwasher-pico C CXX)

pico_sdk_init()

add_executable(dishwasher
        src/main.c
        src/ir/ir_raw.c
        include/ir/ir_raw.h
)

target_include_directories(dishwasher PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/src
        ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(dishwasher pico_stdlib)

# .uf2, .bin, .hex
pico_add_extra_outputs(dishwasher)
