cmake_minimum_required(VERSION 3.22)

set(CMAKE_C_STANDARD 17)
set(CMAKE_CXX_STANDARD 20)

set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# set(FREERTOS_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cube/Middlewares/Third_Party/FreeRTOS/Source)
# set(FREERTOS_SOURCE ${FREERTOS_PATH})

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

set(CMAKE_PROJECT_NAME micras)

set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

set(DEVICE STM32G474RE)
set(BOARD_VERSION "")

if(BOARD_VERSION STREQUAL "")
    set(PROJECT_RELEASE ${CMAKE_PROJECT_NAME})
else()
    set(PROJECT_RELEASE ${CMAKE_PROJECT_NAME}_${BOARD_VERSION})
endif()

message("Build type: " ${CMAKE_BUILD_TYPE})

set(LAUNCH_JSON_PATH ${CMAKE_CURRENT_SOURCE_DIR}/.vscode/launch.json)
set(DEBUG_FILE_NAME ${CMAKE_PROJECT_NAME})

include(cmake/config_validation.cmake)

set(CMAKE_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/cube)
include(cube/cmake/gcc-arm-none-eabi.cmake)
set(CMAKE_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})

project(${CMAKE_PROJECT_NAME} C CXX ASM)

include(cmake/workspace.cmake)
include(cmake/targets.cmake)
include(cmake/utilities.cmake)
include(cmake/linter.cmake)

###############################################################################
## Include directories
###############################################################################

set(PROJECT_INCLUDE_DIRECTORIES
    ./cfg
    ./inc
)

set(TEST_INCLUDE_DIRECTORIES
    ./tests/inc
)

# set(FREE_RTOS_INCLUDE_DIRECTORIES
#     ${FREERTOS_SOURCE}/include
#     ${FREERTOS_SOURCE}/CMSIS_RTOS_V2
#     ${FREERTOS_SOURCE}/portable/GCC/${DEVICE_PORT}
# )

###############################################################################
## Input files
###############################################################################

file(GLOB_RECURSE PROJECT_SOURCES CONFIGURE_DEPENDS "src/*.cpp" "src/*.c" "cfg/*.cpp" "cfg/*.c")
file(GLOB_RECURSE PROJECT_HEADERS CONFIGURE_DEPENDS "inc/*.hpp" "inc/*.h" "cfg/*.hpp" "cfg/*.h")

file(GLOB_RECURSE TESTS_SOURCES CONFIGURE_DEPENDS "tests/src/*.cpp" "tests/src/*.c")
file(GLOB_RECURSE TESTS_HEADERS CONFIGURE_DEPENDS "tests/inc/*.hpp" "tests/inc/*.h")
file(GLOB_RECURSE TESTS_BIN CONFIGURE_DEPENDS "tests/bin/*.cpp" "tests/bin/*.c")

targets_generate_format_target(PROJECT_SOURCES PROJECT_HEADERS TESTS_SOURCES TESTS_HEADERS TESTS_BIN)

list(REMOVE_ITEM PROJECT_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/main.c
)

###############################################################################
## Main executable target
###############################################################################

set(LIB_SOURCES
    lib/lsm6dsv-pid/lsm6dsv_reg.c
)

set(LIB_INCLUDE_DIRECTORIES
    lib/lsm6dsv-pid
)

add_subdirectory(cube/cmake/stm32cubemx)
get_target_property(CUBE_SOURCES stm32cubemx INTERFACE_SOURCES)

# Remove warnings from Cube and library sources
set_source_files_properties(
  ${CUBE_SOURCES}
  ${LIB_SOURCES}
  PROPERTIES
  COMPILE_FLAGS "-w"
)

# Remove warnings from Cube header files
get_target_property(CUBE_HEADERS stm32cubemx INTERFACE_INCLUDE_DIRECTORIES)
target_include_directories(stm32cubemx SYSTEM INTERFACE ${CUBE_HEADERS})

add_library(${CMAKE_PROJECT_NAME}_lib STATIC
    ${PROJECT_SOURCES}
    ${LIB_SOURCES}
)

target_include_directories(${CMAKE_PROJECT_NAME}_lib PUBLIC
    ${PROJECT_INCLUDE_DIRECTORIES}
    ${LIB_INCLUDE_DIRECTORIES}
)

target_link_libraries(${CMAKE_PROJECT_NAME}_lib
    stm32cubemx
)

add_executable(${CMAKE_PROJECT_NAME} "src/main.cpp")

target_link_libraries(${CMAKE_PROJECT_NAME}
    ${CMAKE_PROJECT_NAME}_lib
)

stm32_print_size_of_target(${PROJECT_NAME})
stm32_generate_hex_file(${PROJECT_NAME})

targets_generate_vsfiles_target(${PROJECT_NAME})
targets_generate_flash_target(${PROJECT_NAME})
targets_generate_helpme_target()

###############################################################################
## Generate test executables
###############################################################################

foreach(TEST_FILE ${TESTS_BIN})

    # If TEST_FILE contains /dir1/dir2/file.cpp, TEST_NAME will be 'file'
    get_filename_component(TEST_NAME ${TEST_FILE} NAME_WLE)

    add_executable(${TEST_NAME} EXCLUDE_FROM_ALL
        ${TEST_FILE}
        ${TESTS_SOURCES}
    )

    target_include_directories(${TEST_NAME} PRIVATE
        ${TESTS_HEADERS}
    )

    target_link_libraries(${TEST_NAME}
        ${CMAKE_PROJECT_NAME}_lib
    )

    stm32_generate_hex_file(${TEST_NAME})

    targets_generate_vsfiles_target(${TEST_NAME})
    targets_generate_flash_target(${TEST_NAME})

endforeach()
