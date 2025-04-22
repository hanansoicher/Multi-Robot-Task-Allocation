cmake_minimum_required(VERSION 3.13)
include(lib/pico-sdk/pico_sdk_init.cmake)
set(PICO_BOARD pololu_3pi_2040_robot)
set(PICO_PLATFORM "rp2040")

project(robot_loop)

pico_sdk_init()

add_subdirectory(${CMAKE_SOURCE_DIR}/lib/pololu-3pi-2040-robot/c/pololu_3pi_2040_robot)

add_executable(robot_loop ${CMAKE_SOURCE_DIR}/src/robot/robot_loop.c)
# add_executable(robot_loop ${CMAKE_SOURCE_DIR}/test_connection.c)
target_include_directories(robot_loop PRIVATE 
    ${CMAKE_SOURCE_DIR}/lib/pololu-3pi-2040-robot/c
)

target_link_libraries(robot_loop
    pico_stdlib
    pololu_3pi_2040_robot
)

pico_enable_stdio_usb(robot_loop 1)
pico_enable_stdio_uart(robot_loop 0)

pico_add_extra_outputs(robot_loop)


