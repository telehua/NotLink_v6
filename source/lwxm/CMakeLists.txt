cmake_minimum_required (VERSION 3.24)

project ("lwxm" C)

file(GLOB SOURCE_FILES "*.c")

add_library(${PROJECT_NAME} STATIC
	${SOURCE_FILES}
)

target_link_libraries(${PROJECT_NAME}
	PUBLIC
	"ulog"
)

target_include_directories(${PROJECT_NAME}
	PUBLIC
	"${CMAKE_CURRENT_SOURCE_DIR}"
)
