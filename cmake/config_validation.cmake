# Name: config_validation.cmake file
# ThundeRatz Robotics Team
# Brief: This file contains the existence checks of the declared variables
# 04/2023

###############################################################################
## Existence checks
###############################################################################

# Check if CMake build type is correctly configured
if(NOT (BUILD_TYPE STREQUAL "Release"        OR BUILD_TYPE STREQUAL "Debug" OR
        BUILD_TYPE STREQUAL "RelWithDebInfo" OR BUILD_TYPE STREQUAL "MinSizeRel"))
    set(BUILD_TYPE "RelWithDebInfo")
endif()
set(CMAKE_BUILD_TYPE ${BUILD_TYPE})
message(STATUS "Build type: " ${CMAKE_BUILD_TYPE})

# Detect if running inside WSL
if(DEFINED ENV{WSL_DISTRO_NAME} AND NOT "$ENV{WSL_DISTRO_NAME}" STREQUAL "")
    set(IS_WSL TRUE)
    message(STATUS "WSL detected")
else()
    set(IS_WSL FALSE)
    message(STATUS "Linux detected")
endif()

# Set STM32CubeMX command
if(DEFINED ENV{CUBE_CMD})
    set(CUBE_CMD $ENV{CUBE_CMD})
    message(STATUS "Using STM32CubeMX from environment: ${CUBE_CMD}")
else()
    message(STATUS "CUBE_CMD not defined, trying default path")

    if(IS_WSL)
        set(CUBE_CMD "/mnt/c/Program Files/STMicroelectronics/STM32Cube/STM32CubeMX/STM32CubeMX.exe")
    else()
        set(CUBE_CMD "/usr/local/STMicroelectronics/STM32Cube/STM32CubeMX/STM32CubeMX")
    endif()

    if(EXISTS ${CUBE_CMD})
        message(STATUS "STM32CubeMX found at: ${CUBE_CMD}")
    else()
        message(FATAL_ERROR "STM32CubeMX executable not found at expected path: ${CUBE_CMD}")
    endif()
endif()

# Set J-Link and STM32 Programmer executables
if(IS_WSL)
    set(JLINK_CMD "JLink.exe")
    set(PROGRAMMER_CMD "STM32_Programmer_CLI.exe")
else()
    set(JLINK_CMD "JLinkExe")
    set(PROGRAMMER_CMD "STM32_Programmer_CLI")
endif()

# Check if OpenOCD variables are properly defined
if (DEFINED ENV{OPENOCD_SCRIPTS_PATH})
    message(STATUS "OPENOCD_SCRIPTS_PATH defined as $ENV{OPENOCD_SCRIPTS_PATH}")
    set(OPENOCD_SCRIPTS_PATH $ENV{OPENOCD_SCRIPTS_PATH})
else()
    set(OPENOCD_SCRIPTS_PATH "/usr/share/openocd/scripts")
    message(STATUS "OPENOCD_SCRIPTS_PATH not defined. Using default path ${OPENOCD_SCRIPTS_PATH}")
endif()

# Check if STM32CubeMX project is correctly configured
set(CUBE_CMAKE_TOOLCHAIN_CONFIG "ProjectManager.TargetToolchain=CMake")
set(IOC_FILE cube/${PROJECT_RELEASE}.ioc)
file(READ ${IOC_FILE} IOC_CONTENTS)
string(FIND "${IOC_CONTENTS}" ${CUBE_CMAKE_TOOLCHAIN_CONFIG} CUBE_CMAKE_TOOLCHAIN_CONFIG_POS)
if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${IOC_FILE})
    message(FATAL_ERROR "CubeMX ${IOC_FILE} project file not found")
elseif(${CUBE_CMAKE_TOOLCHAIN_CONFIG_POS} EQUAL -1)
    message(FATAL_ERROR "CMake toolchain not selected in CubeMX project")
else()
    message(STATUS "CMake toolchain selected in CubeMX project")
endif()

# Set DEVICE variable based on cube configuration
string(REGEX MATCH "ProjectManager\.DeviceId=[^\n]+" DEVICE_LINE "${IOC_CONTENTS}")
string(SUBSTRING ${DEVICE_LINE} 24 11 DEVICE)
string(TOLOWER ${DEVICE} LOWERCASE_DEVICE)
string(SUBSTRING ${LOWERCASE_DEVICE} 0 7 TARGET_CFG)
message(STATUS "Device is ${DEVICE}")

# Check cube directory for files
# If it's empty, generate the files
file(GLOB_RECURSE CUBE_SOURCES_CHECK "${CMAKE_CURRENT_SOURCE_DIR}/cube/Src/*.c")
list(LENGTH CUBE_SOURCES_CHECK CUBE_LENGHT)
if(CUBE_LENGHT EQUAL 0)
    message(STATUS "Cube directory is empty. Generating cube files...")

    file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/.cube
        "config load ../cube/${PROJECT_RELEASE}.ioc\n"
        "project generate\n"
        "exit\n"
    )

    execute_process(COMMAND ${CUBE_CMD} -q ${CMAKE_CURRENT_BINARY_DIR}/.cube)
endif()
