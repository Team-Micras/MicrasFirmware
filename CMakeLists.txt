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

# From GCC 14 on, CMake scans every C++20 or later source for module imports, which puts
# -fmodules-ts and a module mapper into the compilation database and makes clang-tidy reject
# every file. No target here declares a module, so the scan is only overhead.
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

# The board layer lives in config/targets/<version>/target.hpp and is selected by the same variable
# that selects the STM32CubeMX project, so that the two can never describe different boards. The
# selection is made by putting one board's directory on the include path, so that "target.hpp"
# resolves to it and nothing has to name the board.
set(MICRAS_TARGET_DIRECTORY "config/targets/${BOARD_VERSION}")

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
#
# Release optimises for speed, not size: the image uses a small fraction of the 1 MiB flash, so
# size is the one resource in surplus. -fno-math-errno is what turns sqrtf and hypotf into FPU
# instructions, since GCC must otherwise preserve the errno side effect, and -ffp-contract=fast
# fuses multiply-adds into VFMA. -ffast-math is deliberately absent: the controllers rely on
# well defined behaviour around zero, and a stray NaN becoming undefined behaviour is the worst
# possible failure mode for a robot.
#
# RelWithDebInfo is Release without -flto, for when a debugger has to make sense of the image;
# MinSizeRel keeps the size oriented flags for a build that has to fit somewhere smaller.
set(MICRAS_MATH_FLAGS "-fno-math-errno -ffp-contract=fast")

set(CMAKE_C_FLAGS_DEBUG     "-Og -g3")
set(CMAKE_CXX_FLAGS_DEBUG   "-Og -g3")
set(CMAKE_C_FLAGS_RELEASE   "-O2 -g3 -flto ${MICRAS_MATH_FLAGS}")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -g3 -flto ${MICRAS_MATH_FLAGS}")
set(CMAKE_C_FLAGS_RELWITHDEBINFO     "-O2 -g3 ${MICRAS_MATH_FLAGS}")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO   "-O2 -g3 ${MICRAS_MATH_FLAGS}")
set(CMAKE_C_FLAGS_MINSIZEREL   "-Os -g3 -flto ${MICRAS_MATH_FLAGS}")
set(CMAKE_CXX_FLAGS_MINSIZEREL "-Os -g3 -flto ${MICRAS_MATH_FLAGS}")

# The link step needs the same optimisation level as the compile step for -flto to take effect,
# and the whole archive wrapping around micras_hal requires the linker plugin to see the LTO
# objects, which the gcc driver arranges as long as the flag is on the link line too.
set(CMAKE_EXE_LINKER_FLAGS_RELEASE     "-O2 -flto")
set(CMAKE_EXE_LINKER_FLAGS_MINSIZEREL  "-Os -flto")

project(${CMAKE_PROJECT_NAME} C CXX ASM)

include(cmake/utilities.cmake)
include(cmake/warnings.cmake)
include(cmake/targets.cmake)
include(cmake/docs.cmake)

# Workaround for the STM32CubeMX CMake generator writing into ${CMAKE_PROJECT_NAME}
# instead of a target of its own (present since 6.14, still present in 6.18.1)
add_library(cube_app OBJECT)
set(CUBE_PROJECT_NAME ${CMAKE_PROJECT_NAME})
set(CMAKE_PROJECT_NAME cube_app)
add_subdirectory(cube/cmake/stm32cubemx)
set(CMAKE_PROJECT_NAME ${CUBE_PROJECT_NAME})

###############################################################################
## Generated tree interface
###############################################################################

# Everything micras_hal needs from the generated CubeMX tree, named here rather than inside the
# package, so that micras_hal names no target it does not own and another project can satisfy the
# dependency by providing the same two names.
#
# The headers and the device macros travel as an interface target, propagated to every consumer
# because config/ and micras_proxy also include main.h. The generated objects cannot: CMake adds
# an object library's files only to the targets that link it *directly*, so routing them through
# an interface target would silently drop them from libmicras_hal.a.
get_target_property(CUBE_INCLUDE_DIRECTORIES stm32cubemx INTERFACE_INCLUDE_DIRECTORIES)
get_target_property(CUBE_COMPILE_DEFINITIONS stm32cubemx INTERFACE_COMPILE_DEFINITIONS)

add_library(micras_cube INTERFACE)
add_library(micras::cube ALIAS micras_cube)

target_include_directories(micras_cube SYSTEM INTERFACE ${CUBE_INCLUDE_DIRECTORIES})
target_compile_definitions(micras_cube INTERFACE ${CUBE_COMPILE_DEFINITIONS})

set(MICRAS_CUBE_OBJECT_LIBRARIES cube_app STM32_Drivers)

# The generated sources are not ours to warn about
foreach(CUBE_OBJECT_LIBRARY ${MICRAS_CUBE_OBJECT_LIBRARIES})
    target_compile_options(${CUBE_OBJECT_LIBRARY} PRIVATE -w)
endforeach()

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
    ${MICRAS_TARGET_DIRECTORY}
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
        ${MICRAS_TARGET_DIRECTORY}
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
