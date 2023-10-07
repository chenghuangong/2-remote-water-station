cmake_minimum_required(VERSION 3.20)

# modify flash speed
set(PICO_BOARD "fly_control_rp2040")

# pull in SDK 相当于将pico_skd_import.cmake文件在这里进行展开，和cpp的include类似
# 在pico_sdk_import中有PICO_SDK_PATH，这个我们在cmake的config environment 中进行设置，可以使用相对或者绝对的地址，导入SDK工具包pico-sdk
include(pico_sdk_import.cmake)

# 设置一个项目名称，后面指的是所使用的语言的名称
# Supported languages include C , CXX (i.e. C++), CUDA , OBJC (i.e. Objective-C), OBJCXX , Fortran , HIP , ISPC , and ASM . 
# By default C and CXX are enabled if no language options are given.
project(pico_examples C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# 添加子目录，每一个子目录下面都要添加CMakeLists.txt文件
add_subdirectory(sensor_test)
add_subdirectory(water_station_main)

# 添加编译器选项
add_compile_options(-Wall
        -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
        -Wno-unused-function # we have some for the docs that aren't called
        -Wno-maybe-uninitialized
        )