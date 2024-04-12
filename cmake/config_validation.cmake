# Name: config_validation.cmake file
# ThundeRatz Robotics Team
# Brief: This file contains the existence checks of the declared variables
# 04/2023

###############################################################################
## Auxiliary Sets
###############################################################################

# This set contains all the variables that must be defined by the user
# It is used to check if all of them are properly defined
set(USER_INPUT_VARIABLES
    DEVICE
    BOARD_VERSION
)

###############################################################################
## Existence checks
###############################################################################

# Check if CMake build type is correctly configured
if(NOT (CMAKE_BUILD_TYPE STREQUAL "Release"        OR CMAKE_BUILD_TYPE STREQUAL "Debug" OR
        CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo" OR CMAKE_BUILD_TYPE STREQUAL "MinSizeRel"))
    set(CMAKE_BUILD_TYPE "RelWithDebInfo")
endif()
message(STATUS "Build type: " ${CMAKE_BUILD_TYPE})

## Check if STM32CubeMX variables are properly defined
if(DEFINED ENV{CUBE_PATH})
    message(STATUS "CUBE_PATH defined as $ENV{CUBE_PATH}")
else()
    message(FATAL_ERROR "CUBE_PATH not defined")
endif()

if(CMAKE_HOST_WIN32)
    set(JAVA_EXE "$ENV{CUBE_PATH}\\STM32CubeMX.exe")
    set(CUBE_JAR "$ENV{CUBE_PATH}\\jre\\bin\\java.exe")
    set(JLINK_EXE JLink.exe)
    if(NOT DEFINED ENV{OPENOCD_SCRIPTS_PATH})
        set(OPENOCD_SCRIPTS_PATH "C:\\msys64\\mingw64\\share\\openocd\\scripts")
    endif()
else()
    set(JAVA_EXE $ENV{CUBE_PATH}/jre/bin/java)
    set(CUBE_JAR $ENV{CUBE_PATH}/STM32CubeMX)
    set(JLINK_EXE JLinkExe)
    if(NOT DEFINED ENV{OPENOCD_SCRIPTS_PATH})
        set(OPENOCD_SCRIPTS_PATH "/usr/share/openocd/scripts")
    endif()
endif()

string(TOLOWER ${DEVICE} LOWERCASE_DEVICE)
string(SUBSTRING ${LOWERCASE_DEVICE} 0 7 TARGET_CFG)

# Check if necessary variables are defined:
foreach(VARIABLE ${USER_INPUT_VARIABLES})
    if(NOT DEFINED ${VARIABLE})
        message(FATAL_ERROR "${VARIABLE} not defined")
    endif()
endforeach()
message(STATUS "All necessary variables defined!")

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

# Check cube directory for files
# If it's empty, generate the files
# It's important to do it before find_package(CMSIS)
file(GLOB_RECURSE CUBE_SOURCES_CHECK "${CMAKE_CURRENT_SOURCE_DIR}/cube/Src/*.c")
list(LENGTH CUBE_SOURCES_CHECK CUBE_LENGHT)
if(CUBE_LENGHT EQUAL 0)
    message(STATUS "Cube directory is empty. Generating cube files...")

    file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/.cube
        "config load ${CMAKE_CURRENT_SOURCE_DIR}/cube/${PROJECT_RELEASE}.ioc\n"
        "project generate\n"
        "exit\n"
    )

    execute_process(COMMAND ${JAVA_EXE} -jar ${CUBE_JAR} -q ${CMAKE_CURRENT_BINARY_DIR}/.cube)
endif()
