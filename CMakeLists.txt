cmake_minimum_required(VERSION 3.25...4.4)

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

# No target declares a module, and the scan puts -fmodules-ts and a module mapper into the
# compilation database, which makes clang-tidy reject every file
set(CMAKE_CXX_SCAN_FOR_MODULES OFF)

###############################################################################
## Project Configuration
###############################################################################

# The .ioc file used to generate the project will be PROJECT_RELEASE.ioc

set(CMAKE_PROJECT_NAME micras)
set(BOARD_VERSION "v1" CACHE STRING "Micras board revision")
set_property(CACHE BOARD_VERSION PROPERTY STRINGS "v1")

if(BOARD_VERSION STREQUAL "")
    set(PROJECT_RELEASE ${CMAKE_PROJECT_NAME})
else()
    set(PROJECT_RELEASE "${CMAKE_PROJECT_NAME}_${BOARD_VERSION}")
endif()

# One board's directory goes on the include path, so that "target.hpp" resolves to it and nothing
# has to name the board. The same variable selects the STM32CubeMX project, so the two always
# describe the same board.
set(MICRAS_TARGET_DIRECTORY "config/targets/${BOARD_VERSION}")

set(LAUNCH_JSON_PATH "${CMAKE_CURRENT_SOURCE_DIR}/.vscode/launch.json")
set(DEBUG_FILE_NAME ${CMAKE_PROJECT_NAME})

include(cmake/config_validation.cmake)

if(MICRAS_CUBE_GENERATED)
    include(cmake/toolchain.cmake)
    project(${CMAKE_PROJECT_NAME} C CXX ASM)
else()
    # The same system the generated toolchain file describes, since the build directory keeps
    # the one it was first configured with
    set(CMAKE_SYSTEM_NAME Generic)
    set(CMAKE_SYSTEM_PROCESSOR arm)
    project(${CMAKE_PROJECT_NAME} NONE)
endif()

include(cmake/utilities.cmake)
include(cmake/warnings.cmake)
include(cmake/targets.cmake)
include(cmake/docs.cmake)

###############################################################################
## Input files
###############################################################################

file(GLOB_RECURSE FORMAT_SOURCES CONFIGURE_DEPENDS "src/*.c*" "tests/*.c*" "micras_*/src/*.c*")
file(GLOB_RECURSE FORMAT_HEADERS CONFIGURE_DEPENDS
    "include/*.h*" "config/*.h*" "tests/*.h*" "micras_*/include/*.h*" "micras_*/include/*.tpp"
)

file(GLOB_RECURSE PROJECT_SOURCES CONFIGURE_DEPENDS "src/*.c*")
file(GLOB_RECURSE PROJECT_TESTS CONFIGURE_DEPENDS "tests/src/*/*.c*")

generate_format_target(FORMAT_SOURCES FORMAT_HEADERS)
generate_helpme_text()
generate_vscode_target()

###############################################################################
## Targets waiting for the generated tree
###############################################################################

if(NOT MICRAS_CUBE_GENERATED)
    generate_bootstrap_targets(${PROJECT_NAME} PROJECT_TESTS)
    return()
endif()

###############################################################################
## Packages
###############################################################################

include(cmake/cube.cmake)

add_subdirectory(micras_core)
add_subdirectory(micras_hal)
add_subdirectory(micras_proxy)
add_subdirectory(micras_nav)

micras_apply_warnings(micras_core micras_hal micras_proxy micras_nav)

generate_lint_target(FORMAT_SOURCES)

###############################################################################
## Main executable target
###############################################################################

add_executable(${PROJECT_NAME}
    ${PROJECT_SOURCES}
)

target_include_directories(${PROJECT_NAME} PRIVATE
    include
    config
    ${MICRAS_TARGET_DIRECTORY}
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    micras::nav
    ${MICRAS_CUBE_OBJECT_LIBRARIES}
)

micras_apply_warnings(${PROJECT_NAME})

generate_map_file(${PROJECT_NAME})
generate_hex_file(${PROJECT_NAME})
print_size_of_target(${PROJECT_NAME})

generate_debug_target(${PROJECT_NAME})
generate_flash_target(${PROJECT_NAME})

###############################################################################
## Test executables
###############################################################################

generate_test_targets(PROJECT_TESTS)
generate_test_all_target(${PROJECT_TESTS})
