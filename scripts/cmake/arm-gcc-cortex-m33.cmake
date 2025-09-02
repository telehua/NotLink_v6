set(MCPU_FLAGS "-mcpu=cortex-m33")
set(VFP_FLAGS "-mfloat-abi=hard -mfpu=fpv5-sp-d16")

include(${CMAKE_CURRENT_LIST_DIR}/arm-gcc-cortex-toolchain.cmake)
