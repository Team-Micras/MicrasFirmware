cmake_minimum_required(VERSION 3.22)

###############################################################################
## CMake Configuration
###############################################################################

set(CMAKE_C_STANDARD 17)
set(CMAKE_CXX_STANDARD 20)

set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

###############################################################################
## Project Configuration
###############################################################################

# The .ioc file used to generate the project will be PROJECT_RELEASE.ioc

set(CMAKE_PROJECT_NAME micras)
set(BOARD_VERSION "")

if(BOARD_VERSION STREQUAL "")
    set(PROJECT_RELEASE ${CMAKE_PROJECT_NAME})
else()
    set(PROJECT_RELEASE "${CMAKE_PROJECT_NAME}_${BOARD_VERSION}")
endif()

###############################################################################
## Toolchain Configuration
###############################################################################

set(LAUNCH_JSON_PATH "${CMAKE_CURRENT_SOURCE_DIR}/.vscode/launch.json")
set(DEBUG_FILE_NAME ${CMAKE_PROJECT_NAME})

include(cmake/config_validation.cmake)

set(CMAKE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/cube")
include(cube/cmake/gcc-arm-none-eabi.cmake)
set(CMAKE_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})

project(${CMAKE_PROJECT_NAME} C CXX ASM)

include(cmake/utilities.cmake)
include(cmake/targets.cmake)

add_subdirectory(cube/cmake/stm32cubemx)
add_subdirectory(micras_core)
add_subdirectory(micras_hal)
add_subdirectory(micras_proxy)
add_subdirectory(micras_nav)

###############################################################################
## Input files
###############################################################################

file(GLOB_RECURSE FORMAT_SOURCES CONFIGURE_DEPENDS "src/*.c*" "tests/*.c*" "micras_*/src/*.c*")
file(GLOB_RECURSE FORMAT_HEADERS CONFIGURE_DEPENDS "include/*.h*" "config/*.h*" "tests/*.h*" "micras_*/include/*.h*")

generate_format_target(FORMAT_SOURCES FORMAT_HEADERS)
generate_lint_target(FORMAT_SOURCES)

file(GLOB_RECURSE PROJECT_SOURCES CONFIGURE_DEPENDS "src/*.c*")
file(GLOB_RECURSE PROJECT_TESTS CONFIGURE_DEPENDS "tests/src/*/*.c*")

###############################################################################
## Main executable target
###############################################################################

add_executable(${PROJECT_NAME}
    ${PROJECT_SOURCES}
)

target_include_directories(${PROJECT_NAME} PUBLIC
    include
    config
)

target_link_libraries(${PROJECT_NAME} PUBLIC
    micras::nav
)

generate_hex_file(${PROJECT_NAME})
print_size_of_target(${PROJECT_NAME})

generate_helpme_text()
generate_vscode_tasks_json()
generate_debug_target(${PROJECT_NAME})
generate_flash_target(${PROJECT_NAME})

###############################################################################
## Generate test executables
###############################################################################

foreach(TEST_FILE ${PROJECT_TESTS})
    get_filename_component(TEST_NAME ${TEST_FILE} NAME_WLE)

    add_executable(${TEST_NAME} EXCLUDE_FROM_ALL
        ${TEST_FILE}
    )

    target_include_directories(${TEST_NAME} PUBLIC
        tests/include
        config
    )

    target_link_libraries(${TEST_NAME} PUBLIC
        micras::nav
    )

    generate_hex_file(${TEST_NAME})
    print_size_of_target(${TEST_NAME})

    generate_debug_target(${TEST_NAME})
    generate_flash_target(${TEST_NAME})
endforeach()

generate_test_all_target(${PROJECT_TESTS})
