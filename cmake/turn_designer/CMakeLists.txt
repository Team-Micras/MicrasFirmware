cmake_minimum_required(VERSION 3.28...4.4)

# The turn designer for the robot build, which cross-compiles: the firmware's build adds this project
# as an external project, with the host compilers, and it designs the turns into the include directory
# it is given. The simulation's build compiles the designer itself.
project(micras_turn_designer C CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_SCAN_FOR_MODULES OFF)

set(BOARD_VERSION "v1" CACHE STRING "Micras board revision")
set(MICRAS_TWO_BEND_TURNS_DIRECTORY "" CACHE PATH "Where the designer writes two_bend_turns.hpp")

if(MICRAS_TWO_BEND_TURNS_DIRECTORY STREQUAL "")
    message(FATAL_ERROR "MICRAS_TWO_BEND_TURNS_DIRECTORY is not set")
endif()

get_filename_component(MICRAS_FIRMWARE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)

# The designer only needs the navigation library
set(MICRAS_LIB_HAL OFF CACHE BOOL "" FORCE)
set(MICRAS_LIB_COMM OFF CACHE BOOL "" FORCE)
set(MICRAS_LIB_TESTS OFF CACHE BOOL "" FORCE)
set(MICRAS_LIB_DOCS OFF CACHE BOOL "" FORCE)
add_subdirectory("${MICRAS_FIRMWARE_ROOT}/external/micras-lib" micras-lib EXCLUDE_FROM_ALL)

include("${MICRAS_FIRMWARE_ROOT}/cmake/app_sources.cmake")
include("${MICRAS_FIRMWARE_ROOT}/cmake/two_bend_turns.cmake")

add_executable(micras_turn_designer
    "${MICRAS_FIRMWARE_ROOT}/sim/tools/turn_designer.cpp"
)

target_include_directories(micras_turn_designer PRIVATE
    ${MICRAS_APP_INCLUDE_DIRECTORIES}
)

target_link_libraries(micras_turn_designer PRIVATE
    micras::nav
)

micras_add_two_bend_turns(micras_turn_designer "${MICRAS_TWO_BEND_TURNS_DIRECTORY}")
