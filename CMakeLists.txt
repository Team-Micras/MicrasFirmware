cmake_minimum_required(VERSION 3.28...4.4)

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

# micras-lib has a switch of its own, MICRAS_LIB_WERROR
option(MICRAS_WERROR "Treat the warnings of the firmware's own targets as errors" OFF)

###############################################################################
## Host dispatch
###############################################################################

# The host backend builds the simulation of sim/ with the host compilers, before the board
# validation, the .ioc parsing and the ARM toolchain. It never includes the toolchain file, so the
# exceptions and RTTI the simulator's firmware thread unwinds with stay on.
set(MICRAS_LIB_HAL_BACKEND "stm32" CACHE STRING "stm32 builds the robot image, host the simulation")
set_property(CACHE MICRAS_LIB_HAL_BACKEND PROPERTY STRINGS stm32 host)

if(MICRAS_LIB_HAL_BACKEND STREQUAL "host")
    project(micras_host C CXX)
    enable_testing()
    add_subdirectory(sim)
    return()
endif()

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
include(cmake/targets.cmake)
include(cmake/docs.cmake)

###############################################################################
## Input files
###############################################################################

# The firmware's own sources: the libraries are formatted and linted in micras-lib
file(GLOB_RECURSE FORMAT_SOURCES CONFIGURE_DEPENDS "src/*.cpp" "config/*.cpp" "tests/*.cpp")
file(GLOB_RECURSE FORMAT_HEADERS CONFIGURE_DEPENDS "include/*.hpp" "config/*.hpp" "tests/*.hpp")

include(cmake/app_sources.cmake)
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

# After the bootstrap return, where a project() would cache the host compilers, and after the board
# target it links. micras-lib applies its warnings to its own targets.
add_subdirectory(external/micras-lib EXCLUDE_FROM_ALL)

include(external/micras-lib/cmake/micras_warnings.cmake)

generate_lint_target(FORMAT_SOURCES)

###############################################################################
## Configuration computed when the firmware is compiled
###############################################################################

# The turns are designed in one source, compiled once for the executable and every test, since
# designing them takes the compiler most of a gigabyte and several seconds
add_library(micras_config OBJECT
    ${MICRAS_APP_CONFIG_SOURCES}
)

target_include_directories(micras_config PRIVATE
    ${MICRAS_APP_INCLUDE_DIRECTORIES}
)

target_link_libraries(micras_config PRIVATE
    micras::nav
)

micras_apply_warnings(micras_config WERROR ${MICRAS_WERROR})

###############################################################################
## Main executable target
###############################################################################

add_executable(${PROJECT_NAME}
    ${MICRAS_APP_SOURCES}
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${MICRAS_APP_INCLUDE_DIRECTORIES}
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    micras_config
    micras::nav
    micras::comm
    micras::proxy
    ${MICRAS_CUBE_OBJECT_LIBRARIES}
)

micras_apply_warnings(${PROJECT_NAME} WERROR ${MICRAS_WERROR})

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
