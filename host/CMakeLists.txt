cmake_minimum_required(VERSION 3.20)
project(f401_host LANGUAGES C CXX)

# C++20：span / optional / numbers / concepts 都在这个标准里。
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

option(F401_BUILD_TESTS "Build host tests" ON)
option(F401_SANITIZE "Enable AddressSanitizer/UBSan" OFF)

if(F401_SANITIZE)
    add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
    add_link_options(-fsanitize=address,undefined)
endif()

# 契约层：只有头文件，固件 C 和宿主 C++ 共用。
add_library(f401_protocol INTERFACE)
target_include_directories(f401_protocol INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/../protocol)

# 纯算法层：运动学、里程计；不依赖串口/ROS。
add_library(f401_core INTERFACE)
target_include_directories(f401_core INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(f401_core INTERFACE f401_protocol)

# HAL 接口层：IBase 等纯虚接口。
add_library(f401_hal INTERFACE)
target_include_directories(f401_hal INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(f401_hal INTERFACE f401_core)

# 驱动层：串口 RAII、Link v2 编解码、SerialBase。
add_library(f401_drivers STATIC
    src/drivers/serial_port.cpp
    src/drivers/link_v2.cpp
    src/drivers/serial_base.cpp
)
target_include_directories(f401_drivers PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(f401_drivers PUBLIC f401_hal)
target_compile_options(f401_drivers PRIVATE -Wall -Wextra -Wpedantic -Werror)

# 宿主应用（骨架验收程序）。
add_executable(f401_host src/app/main.cpp)
target_link_libraries(f401_host PRIVATE f401_drivers)
target_compile_options(f401_host PRIVATE -Wall -Wextra -Wpedantic -Werror)

if(F401_BUILD_TESTS)
    enable_testing()

    add_executable(f401_codec_test tests/codec_test.cpp)
    target_link_libraries(f401_codec_test PRIVATE f401_drivers)
    target_include_directories(f401_codec_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tests)
    add_test(NAME codec COMMAND f401_codec_test)

    add_executable(f401_core_test tests/core_test.cpp)
    target_link_libraries(f401_core_test PRIVATE f401_drivers)
    target_include_directories(f401_core_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tests)
    add_test(NAME core COMMAND f401_core_test)

    add_executable(f401_serial_port_test tests/serial_port_test.cpp)
    target_link_libraries(f401_serial_port_test PRIVATE f401_drivers)
    target_include_directories(f401_serial_port_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tests)
    add_test(NAME serial_port COMMAND f401_serial_port_test)

    # 共享契约头文件的 C11 编译检查。
    add_executable(f401_c_header_test tests/c_header_test.c)
    target_link_libraries(f401_c_header_test PRIVATE f401_protocol)
    add_test(NAME c_header COMMAND f401_c_header_test)
endif()
