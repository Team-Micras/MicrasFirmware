cmake_minimum_required(VERSION 3.22...4.4)

###############################################################################
## CMake Configuration
###############################################################################

set(CMAKE_C_STANDARD 17)
set(CMAKE_CXX_STANDARD 23)

set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

###############################################################################
## Project Configuration
###############################################################################

# The .ioc file used to generate the project will be PROJECT_RELEASE.ioc

set(CMAKE_PROJECT_NAME micras)
set(BOARD_VERSION "v1" CACHE STRING "Micras board revision")
set_property(CACHE BOARD_VERSION PROPERTY STRINGS "v0" "v1")

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

# The toolchain file hardcodes a single map file name, which every target would overwrite.
# It is removed here so that generate_map_file() can give each target its own.
string(REGEX REPLACE " *-Wl,-Map=[^ ]*" "" CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS}")

# The toolchain file ships -O0 -g3 for Debug and -Os -g0 for Release. Debug information is not
# loaded onto the target, so it costs no flash and is kept in every configuration, which is what
# makes a crash from a competition run symbolisable. Debug uses -Og instead of -O0 so that the
# robot still runs fast enough to be worth debugging.
set(CMAKE_C_FLAGS_DEBUG     "-Og -g3")
set(CMAKE_CXX_FLAGS_DEBUG   "-Og -g3")
set(CMAKE_C_FLAGS_RELEASE   "-Os -g3")
set(CMAKE_CXX_FLAGS_RELEASE "-Os -g3")

project(${CMAKE_PROJECT_NAME} C CXX ASM)

include(cmake/utilities.cmake)
include(cmake/warnings.cmake)
include(cmake/targets.cmake)

# Workaround for the STM32CubeMX CMake generator writing into ${CMAKE_PROJECT_NAME}
# instead of a target of its own (present since 6.14, still present in 6.18.1)
add_library(cube_app OBJECT)
set(CUBE_PROJECT_NAME ${CMAKE_PROJECT_NAME})
set(CMAKE_PROJECT_NAME cube_app)
add_subdirectory(cube/cmake/stm32cubemx)
set(CMAKE_PROJECT_NAME ${CUBE_PROJECT_NAME})

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

target_include_directories(${PROJECT_NAME} PRIVATE
    include
    config
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    micras::nav
    micras::warnings
)

generate_map_file(${PROJECT_NAME})
generate_hex_file(${PROJECT_NAME})
print_size_of_target(${PROJECT_NAME})

generate_helpme_text()
generate_vscode_target()
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

    target_include_directories(${TEST_NAME} PRIVATE
        tests/include
        config
    )

    target_link_libraries(${TEST_NAME} PRIVATE
        micras::nav
        micras::warnings
    )

    generate_map_file(${TEST_NAME})
    generate_hex_file(${TEST_NAME})
    print_size_of_target(${TEST_NAME})

    generate_debug_target(${TEST_NAME})
    generate_flash_target(${TEST_NAME})
endforeach()

generate_test_all_target(${PROJECT_TESTS})
