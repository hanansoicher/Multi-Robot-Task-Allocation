cmake_minimum_required(VERSION 3.13)

set(PICO_SDK_PATH "${CMAKE_SOURCE_DIR}/pololu-3pi-2040-robot/c/pico-sdk")
set(PICO_BOARD pololu_3pi_2040_robot)
set(PICO_PLATFORM "rp2040")
include(${PICO_SDK_PATH}/pico_sdk_init.cmake)


project(robot_ble C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_subdirectory(${CMAKE_SOURCE_DIR}/pololu-3pi-2040-robot/c/pololu_3pi_2040_robot)

add_executable(robot_ble robot_loop_feedback_control.c)

target_include_directories(robot_ble PRIVATE 
    ${CMAKE_SOURCE_DIR}/pololu-3pi-2040-robot/c
)

target_link_libraries(robot_ble
    pico_stdlib
    pololu_3pi_2040_robot
)

pico_enable_stdio_usb(robot_ble 1)
pico_enable_stdio_uart(robot_ble 0)

pico_add_extra_outputs(robot_ble)


