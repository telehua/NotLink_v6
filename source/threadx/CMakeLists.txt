# 项目名称
project("threadx" C ASM)

# 添加源码
add_library(${PROJECT_NAME} STATIC
    # "${CMAKE_CURRENT_SOURCE_DIR}/ports/cortex_m7/gnu/tx_initialize_low_level.S"
)

target_compile_definitions(${PROJECT_NAME}
PRIVATE
    "TX_INCLUDE_USER_DEFINE_FILE"
)

add_subdirectory("common")
add_subdirectory("ports/cortex_m7/gnu")

# 链接库
target_link_libraries(${PROJECT_NAME}
    PUBLIC
    "bsp"
)

# 头文件路径
target_include_directories(${PROJECT_NAME}
    PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}"
)
