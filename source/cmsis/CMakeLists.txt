# 项目名称
project("cmsis" C ASM)

# head only
add_library(${PROJECT_NAME}
    INTERFACE
)

# 头文件路径
target_include_directories(${PROJECT_NAME}
    INTERFACE
    "${CMAKE_CURRENT_SOURCE_DIR}/Include"
    "${CMAKE_CURRENT_SOURCE_DIR}/Device/ST/STM32H7xx/Include"
)
