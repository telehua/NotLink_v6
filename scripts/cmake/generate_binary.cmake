function(generate_binary)
    # 生成bin文件
    add_custom_target(${PROJECT_NAME}_binary_bin
        DEPENDS "${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin")
    add_custom_command(
        OUTPUT "${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin"
        DEPENDS ${PROJECT_NAME}
        COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}> "${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin"
    )

    # 生成hex文件
    add_custom_target(${PROJECT_NAME}_binary_hex
        DEPENDS "${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex")
    add_custom_command(
        OUTPUT "${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex"
        DEPENDS ${PROJECT_NAME}
        COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${PROJECT_NAME}> "${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex"
    )

    # 添加到依赖
    add_custom_target(${PROJECT_NAME}_binaries ALL DEPENDS ${PROJECT_NAME}
        DEPENDS ${PROJECT_NAME}_binary_bin
        DEPENDS ${PROJECT_NAME}_binary_hex
    )
endfunction()
