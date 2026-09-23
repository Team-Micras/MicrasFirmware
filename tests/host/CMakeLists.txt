# Tests for the parts that have no hardware behind them: the variable pool, the flash image, the
# framing and the whole session layer. They build with the host compiler and run in the terminal,
# which is what makes them worth writing at all, and is configured separately from the firmware
# because it is a different toolchain:
#
#   cmake -S tests/host -B build/host && cmake --build build/host && ctest --test-dir build/host

cmake_minimum_required(VERSION 3.25...4.4)

project(micras_host_tests CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(MICRAS_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../..)

enable_testing()

add_library(micras_host_core STATIC
    ${MICRAS_ROOT}/micras_core/src/cobs.cpp
    ${MICRAS_ROOT}/micras_core/src/variable_pool.cpp
)

target_include_directories(micras_host_core PUBLIC ${MICRAS_ROOT}/micras_core/include)

add_library(micras_host_comm STATIC
    ${MICRAS_ROOT}/micras_comm/src/frame.cpp
    ${MICRAS_ROOT}/micras_comm/src/link.cpp
)

target_include_directories(micras_host_comm PUBLIC ${MICRAS_ROOT}/micras_comm/include)
target_link_libraries(micras_host_comm PUBLIC micras_host_core)

# The flash of the storage is stood in for by a buffer that can be told to stop accepting writes,
# which is the only way to test what an interrupted save leaves behind
add_library(micras_host_proxy STATIC
    ${MICRAS_ROOT}/micras_proxy/src/storage.cpp
)

target_include_directories(micras_host_proxy PUBLIC
    ${MICRAS_ROOT}/micras_proxy/include
    ${CMAKE_CURRENT_SOURCE_DIR}/stub
)

target_link_libraries(micras_host_proxy PUBLIC micras_host_core)

foreach(TEST_NAME variable_pool crc cobs frame storage link)
    add_executable(test_${TEST_NAME} test_${TEST_NAME}.cpp)
    target_include_directories(test_${TEST_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    target_compile_options(test_${TEST_NAME} PRIVATE -Wall -Wextra -fsanitize=address,undefined)
    target_link_options(test_${TEST_NAME} PRIVATE -fsanitize=address,undefined)
    add_test(NAME ${TEST_NAME} COMMAND test_${TEST_NAME})
endforeach()

target_link_libraries(test_variable_pool PRIVATE micras_host_core)
target_link_libraries(test_crc PRIVATE micras_host_core)
target_link_libraries(test_cobs PRIVATE micras_host_core)
target_link_libraries(test_frame PRIVATE micras_host_comm)
target_link_libraries(test_storage PRIVATE micras_host_proxy)
target_link_libraries(test_link PRIVATE micras_host_comm)
