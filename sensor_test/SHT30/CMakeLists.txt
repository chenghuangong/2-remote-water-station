add_executable(sht30_test main.cpp)

target_link_libraries(sht30_test pico_stdlib hardware_i2c)

pico_add_extra_outputs(sht30_test)

# enable
pico_enable_stdio_usb(sht30_test 1)
# disable
pico_enable_stdio_uart(sht30_test 0)