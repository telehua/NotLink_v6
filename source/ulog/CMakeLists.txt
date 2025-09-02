# 项目名称
project("ulog" C)

# 添加源码
add_library(${PROJECT_NAME} STATIC
    "ulog.c"
)

# 头文件路径
target_include_directories(${PROJECT_NAME}
    PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}"
)

# 链接库
target_link_libraries(${PROJECT_NAME}
    PUBLIC
)
