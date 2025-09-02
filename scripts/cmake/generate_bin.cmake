function(generate_bin)
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}> "${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin"
    )
endfunction()
