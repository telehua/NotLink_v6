# 项目名称
project("bsp" C ASM)

# 寻找源文件
file(GLOB SOURCE_FILES "*.c")

# 添加源码
add_library(${PROJECT_NAME}
    ${SOURCE_FILES}
)

# 链接库
target_link_libraries(${PROJECT_NAME}
    PUBLIC
    "cmsis"
    "hal"
)

# 头文件路径
target_include_directories(${PROJECT_NAME}
    PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}"
)
