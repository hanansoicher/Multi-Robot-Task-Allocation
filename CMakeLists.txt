cmake_minimum_required(VERSION 3.13...3.27)


# initialize pico-sdk from submodule
# note: this must happen before project()
include(pico-sdk/pico_sdk_init.cmake)

project(multi-robot-collaboration)

pico_sdk_init()

add_subdirectory(simpleble ${CMAKE_BINARY_DIR}/simpleble)

add_executable(coordinator_ble_init src/bluetooth/coordinator_ble_init.c)
target_link_libraries(coordinator_ble_init)

add_executable(robot_ble_init src/bluetooth/robot_ble_init.c)

target_link_libraries(robot_ble_init 
    pico_stdlib 
    hardware_uart
    simpleble::simpleble
)

add_executable(receive_assignments src/movement/receive_assignments.c)

target_link_libraries(receive_assignments 
    pico_stdlib 
    hardware_uart
)

add_executable(send_assignments src/movement/send_assignments.c)

target_link_libraries(send_assignments 
    pico_stdlib 
    hardware_uart
)


pico_enable_stdio_usb(robot_ble_init 1) # Enable for debugging
pico_enable_stdio_uart(robot_ble_init 0)  # Disable UART stdout as we're using it for the HM-10