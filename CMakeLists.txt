cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
# set(CMAKE_C_STANDARD 11) #default

set(CMAKE_C_STANDARD 17) #custom
set(CMAKE_CXX_STANDARD 20) #custom

set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# # Cube generated FreeRTOS path (custom)
# set(FREERTOS_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cube/Middlewares/Third_Party/FreeRTOS/Source)
# set(FREERTOS_SOURCE ${FREERTOS_PATH})

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME micras)



# # # Include toolchain file

set(CMAKE_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/cube)

include("cube/cmake/gcc-arm-none-eabi.cmake")
# set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/cube/cmake/gcc-arm-none-eabi.cmake)

set(CMAKE_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Enable CMake support for ASM and C languages
# enable_language(C ASM) #default
# enable_language(C CXX ASM) #custom

# Core project settings
# project(${CMAKE_PROJECT_NAME}) #default
project(${CMAKE_PROJECT_NAME} C CXX ASM) #custom

# Set the board version to an empty string if your board doesn't have a version
set(BOARD_VERSION "")

if(BOARD_VERSION STREQUAL "")
    set(PROJECT_RELEASE ${PROJECT_NAME})
else()
    set(PROJECT_RELEASE ${PROJECT_NAME}_${BOARD_VERSION})
endif()
set(TARGET_BOARD target_${PROJECT_RELEASE})

message("Build type: " ${CMAKE_BUILD_TYPE})

#init custom

set(LAUNCH_JSON_PATH ${CMAKE_CURRENT_SOURCE_DIR}/.vscode/launch.json)
set(DEBUG_FILE_NAME ${PROJECT_NAME})

include(cmake/config_validation.cmake)

# Include toolchain file
# include("cube/cmake/gcc-arm-none-eabi.cmake")

# set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/cube/cmake/gcc-arm-none-eabi.cmake)

include(cmake/workspace.cmake)
include(cmake/targets.cmake)

# Add libraries .c files here
set(LIB_SOURCES
    lib/lsm6dsv-pid/lsm6dsv_reg.c
)

# Add libraries include directories here
set(LIB_INCLUDE_DIRECTORIES
    lib/lsm6dsv-pid
)

###############################################################################
## Include directories
###############################################################################

set(PROJECT_INCLUDE_DIRECTORIES
    ./cfg
    ./inc
    # ./cube/Inc
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

# file(GLOB_RECURSE CUBE_SOURCES CONFIGURE_DEPENDS "cube/Src/*.c")

# # Remove system_xxx.c file from CubeMX, as it is already included in CMSIS
# string(TOLOWER ${DEVICE_FAMILY} DEVICE_FAMILY_LOWER)
# list(REMOVE_ITEM CUBE_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/cube/Src/system_${DEVICE_FAMILY_LOWER}.c)

# Add here the headers to be included in all files
set(FORCED_INCLUDE_HEADERS
)

targets_generate_format_target(PROJECT_SOURCES PROJECT_HEADERS TESTS_SOURCES TESTS_HEADERS TESTS_BIN)


###############################################################################
## Main executable target
###############################################################################

# end custom

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cube/cmake/stm32cubemx)

# # Link directories setup
# target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
#     # Add user defined library search paths
# )

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    ${PROJECT_SOURCES}
    ${LIB_SOURCES}
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    ${PROJECT_INCLUDE_DIRECTORIES}
    ${LIB_INCLUDE_DIRECTORIES}
)

# custom
target_precompile_headers(${PROJECT_NAME} PRIVATE
    ${FORCED_INCLUDE_HEADERS}
)

# # Add project symbols (macros)
# target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
#     # Add user defined symbols
# )

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

# stm32_print_size_of_target(${PROJECT_NAME})
# stm32_generate_hex_file(${PROJECT_NAME})

targets_generate_vsfiles_target(${PROJECT_NAME})
targets_generate_flash_target(${PROJECT_NAME})
targets_generate_helpme_target()




###############################################################################
## Generate test executables
###############################################################################

# Since each test has its own main function, we don't need the main.cpp from the project
list(REMOVE_ITEM PROJECT_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/main.c
)

foreach(TEST_FILE ${TESTS_BIN})

    # If TEST_FILE contains /dir1/dir2/file.cpp, TEST_NAME will be 'file'
    get_filename_component(TEST_NAME ${TEST_FILE} NAME_WLE)

    add_executable(${TEST_NAME} EXCLUDE_FROM_ALL
        ${TEST_FILE}
        # ${CUBE_SOURCES}
        ${PROJECT_SOURCES}
        ${TESTS_SOURCES}
        ${LIB_SOURCES}
    )

    # Add STM32CubeMX generated sources
    # add_subdirectory(cube/cmake/stm32cubemx)

    target_include_directories(${TEST_NAME} PUBLIC
        ${PROJECT_INCLUDE_DIRECTORIES}
        ${LIB_INCLUDE_DIRECTORIES}
    )

    target_precompile_headers(${TEST_NAME} PUBLIC
        ${FORCED_INCLUDE_HEADERS}
    )

    target_link_libraries(${TEST_NAME}
        stm32cubemx
    )

    # stm32_generate_hex_file(${TEST_NAME})

    targets_generate_vsfiles_target(${TEST_NAME})
    targets_generate_flash_target(${TEST_NAME})

endforeach()
