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
    ${MICRAS_ROOT}/micras_comm/src/credit_window.cpp
    ${MICRAS_ROOT}/micras_comm/src/frame.cpp
    ${MICRAS_ROOT}/micras_comm/src/link.cpp
)

target_include_directories(micras_host_comm PUBLIC ${MICRAS_ROOT}/micras_comm/include)
target_link_libraries(micras_host_comm PUBLIC micras_host_core)

# The flash of the storage is stood in for by a buffer that can be told to stop accepting writes,
# which is the only way to test what an interrupted save leaves behind, and the cycle counter by one
# the test winds, which is how the microsecond clock is taken past its wrap
add_library(micras_host_proxy STATIC
    ${MICRAS_ROOT}/micras_proxy/src/storage.cpp
    ${MICRAS_ROOT}/micras_proxy/src/microsecond_clock.cpp
)

target_include_directories(micras_host_proxy PUBLIC
    ${MICRAS_ROOT}/micras_proxy/include
    ${CMAKE_CURRENT_SOURCE_DIR}/stub
)

target_link_libraries(micras_host_proxy PUBLIC micras_host_core)

# The table of the commands each state accepts, which is plain data over the ids of the states
add_library(micras_host_command STATIC
    ${MICRAS_ROOT}/src/command.cpp
)

target_include_directories(micras_host_command PUBLIC ${MICRAS_ROOT}/include)
target_link_libraries(micras_host_command PUBLIC micras_host_core)

# The map of the walls, whose revision the link publishes
add_library(micras_host_nav STATIC
    ${MICRAS_ROOT}/micras_core/src/vector.cpp
    ${MICRAS_ROOT}/micras_nav/src/grid_pose.cpp
)

target_include_directories(micras_host_nav PUBLIC ${MICRAS_ROOT}/micras_nav/include)
target_link_libraries(micras_host_nav PUBLIC micras_host_core)

# The braking of a run and of the calibration of the gyroscope, over the dynamics of the robot and the
# shape of its turns, which the firmware designs when it is compiled
add_library(micras_host_motion STATIC
    ${MICRAS_ROOT}/config/dynamics_config.cpp
    ${MICRAS_ROOT}/micras_nav/src/curve_speed.cpp
    ${MICRAS_ROOT}/micras_nav/src/executor.cpp
    ${MICRAS_ROOT}/micras_nav/src/gyroscope_calibration.cpp
    ${MICRAS_ROOT}/micras_nav/src/line.cpp
    ${MICRAS_ROOT}/micras_nav/src/motion_limits.cpp
    ${MICRAS_ROOT}/micras_nav/src/speed_profile.cpp
    ${MICRAS_ROOT}/micras_nav/src/state.cpp
    ${MICRAS_ROOT}/micras_nav/src/velocity_planner.cpp
)

target_include_directories(micras_host_motion PUBLIC
    ${MICRAS_ROOT}/config
    ${MICRAS_ROOT}/config/targets/v1
)
target_link_libraries(micras_host_motion PUBLIC micras_host_nav)

foreach(TEST_NAME variable_pool cobs frame storage microsecond_clock link credit_window command fsm maze braking)
    add_executable(test_${TEST_NAME} test_${TEST_NAME}.cpp)
    target_include_directories(test_${TEST_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    target_compile_options(test_${TEST_NAME} PRIVATE -Wall -Wextra -fsanitize=address,undefined)
    target_link_options(test_${TEST_NAME} PRIVATE -fsanitize=address,undefined)
    add_test(NAME ${TEST_NAME} COMMAND test_${TEST_NAME})
endforeach()

target_link_libraries(test_variable_pool PRIVATE micras_host_core)
target_link_libraries(test_cobs PRIVATE micras_host_core)
target_link_libraries(test_frame PRIVATE micras_host_comm)

# The frame vectors are also kept as JSON for micras-monitor, and this fails when the copy is stale
add_test(NAME frame_vectors COMMAND test_frame ${CMAKE_CURRENT_SOURCE_DIR}/vectors/v2.json)
target_link_libraries(test_storage PRIVATE micras_host_proxy)
target_link_libraries(test_microsecond_clock PRIVATE micras_host_proxy)
target_link_libraries(test_link PRIVATE micras_host_comm)
target_link_libraries(test_credit_window PRIVATE micras_host_comm)
target_link_libraries(test_command PRIVATE micras_host_command)
target_link_libraries(test_fsm PRIVATE micras_host_core)
target_link_libraries(test_maze PRIVATE micras_host_nav)
target_link_libraries(test_braking PRIVATE micras_host_motion)
