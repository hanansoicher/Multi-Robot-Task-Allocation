### `lib\pololu-3pi-2040-robot\c\pololu_3pi_2040_robot\CMakeLists.txt`

add_library(pololu_3pi_2040_robot
  battery.c
  button.c
  display.c
  font_8x8.c
  font_8x16.c
  ir_sensors.c
  motors.c
  rgb_leds.c
  sh1106.c
  yellow_led.c
  imu.c
)
pico_generate_pio_header(pololu_3pi_2040_robot
  ${CMAKE_CURRENT_LIST_DIR}/qtr_sensor_counter.pio
)
pico_generate_pio_header(pololu_3pi_2040_robot
  ${CMAKE_CURRENT_LIST_DIR}/quadrature_encoder.pio
)
target_include_directories(pololu_3pi_2040_robot PUBLIC include)
target_link_libraries(pololu_3pi_2040_robot
  pico_stdlib
  hardware_adc
  hardware_pio
  hardware_pwm
  hardware_spi
  hardware_i2c
)