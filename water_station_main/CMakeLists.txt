add_executable(water_station_main main.cpp)

target_link_libraries(water_station_main pico_stdlib hardware_i2c hardware_pwm)

pico_add_extra_outputs(water_station_main)

# enable
pico_enable_stdio_usb(water_station_main 0)
# disable
pico_enable_stdio_uart(water_station_main 1)