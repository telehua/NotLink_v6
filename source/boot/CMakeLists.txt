# 最低版本限制
cmake_minimum_required(VERSION 3.24)

# 引入自定义CMAKE函数
include("${CMAKE_SOURCE_DIR}/scripts/cmake/set_target_linker.cmake")
include("${CMAKE_SOURCE_DIR}/scripts/cmake/generate_hex.cmake")
include("${CMAKE_SOURCE_DIR}/scripts/cmake/generate_bin.cmake")
include("${CMAKE_SOURCE_DIR}/scripts/cmake/print_all_variables.cmake")

# 项目名称
project("boot" C ASM)

# 添加源码
add_executable(${PROJECT_NAME}
    "main.c"
    "syscalls.c"
    "sysmem.c"
    "system_stm32h7xx.c"
    "startup_stm32h750xx.c"
    "stm32h7xx_it.c"
)

# 链接库
target_link_libraries(${PROJECT_NAME}
    PRIVATE
    "cmsis"
    "hal"
    "bsp"
    "device"
    "ulog"
    "lwxm"
)

# 头文件路径
target_include_directories(${PROJECT_NAME}
    PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}"
)

# 添加链接脚本
set_target_linker(${PROJECT_NAME} 
    ${CMAKE_SOURCE_DIR}/scripts/ldscripts/STM32H750VBTX_BOOT_FLASH.ld
)

# 生成二进制文件
generate_hex()
generate_bin()
