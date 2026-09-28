###############################################################################
## Micras: the firmware as it is, over the host micras_hal
###############################################################################

# Uses the engine only through its CMake targets (micras::sim_engine, micras::sim_app)
# and the host HAL sources the root exports.

set(MICRAS_FIRMWARE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/MicrasFirmware")

# The commit is read at build time, not here, so meta.json never names a firmware
# the binary was not built from.
set(MICRAS_FIRMWARE_SHA_HEADER "${CMAKE_CURRENT_BINARY_DIR}/generated/micras_firmware_sha.hpp")

add_custom_target(micras_firmware_sha
    COMMAND "${CMAKE_COMMAND}" -DFIRMWARE_DIR=${MICRAS_FIRMWARE_DIR} -DOUTPUT=${MICRAS_FIRMWARE_SHA_HEADER}
            -P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/firmware_sha.cmake"
    BYPRODUCTS "${MICRAS_FIRMWARE_SHA_HEADER}"
    COMMENT "Reading the firmware commit"
    VERBATIM
)

###############################################################################
## ST's register driver of the LSM6DSV and the chip models
###############################################################################

# The commit of v5.1.1, the tag the firmware fetches. The IMU proxy's configuration names the
# driver's enums, and the proxy drives the chip through the driver's source.
include(FetchContent)
FetchContent_Declare(lsm6dsv
    URL      "https://github.com/STMicroelectronics/lsm6dsv-pid/archive/6ede05a3fde7eae80bc1b47affc08257191c1aba.tar.gz"
    URL_HASH "SHA256=bed616e902886809054b34ed25d5af0624542f00ef9bb2e171fe9c525844d064"
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    SYSTEM
)
FetchContent_MakeAvailable(lsm6dsv)

add_library(lsm6dsv_headers INTERFACE)
target_include_directories(lsm6dsv_headers SYSTEM INTERFACE "${lsm6dsv_SOURCE_DIR}")

# The SPI chips the bindings attach to the host HAL's buses: micras::hal_host_models.
add_subdirectory("${PROJECT_SOURCE_DIR}/hal_host/models" "${CMAKE_CURRENT_BINARY_DIR}/hal_host_models")

###############################################################################
## The firmware, compiled for the host
###############################################################################

# Everything but micras_hal, which hal_host replaces, and FmacFilter, which is unused
# and needs the FMAC. The proxies of the two SPI chips talk to the chip models over the
# host SPI, through ST's register driver of the LSM6DSV, which is C. The firmware's own
# main is compiled under another name, and the simulator runs it on the firmware thread.
enable_language(C)

file(GLOB MICRAS_FIRMWARE_SOURCES CONFIGURE_DEPENDS
    "${MICRAS_FIRMWARE_DIR}/micras_core/src/*.cpp"
    "${MICRAS_FIRMWARE_DIR}/micras_nav/src/*.cpp"
    "${MICRAS_FIRMWARE_DIR}/micras_comm/src/*.cpp"
    "${MICRAS_FIRMWARE_DIR}/micras_proxy/src/*.cpp"
    "${MICRAS_FIRMWARE_DIR}/src/*.cpp"
    "${MICRAS_FIRMWARE_DIR}/src/states/*.cpp"
    "${MICRAS_FIRMWARE_DIR}/config/*.cpp"
)
list(REMOVE_ITEM MICRAS_FIRMWARE_SOURCES "${MICRAS_FIRMWARE_DIR}/micras_proxy/src/fmac_filter.cpp")
set_source_files_properties("${MICRAS_FIRMWARE_DIR}/src/main.cpp"
    PROPERTIES COMPILE_DEFINITIONS "main=micras_firmware_main"
)

# The firmware silences the vendor driver's warnings the same way.
set(MICRAS_LSM6DSV_SOURCES "${lsm6dsv_SOURCE_DIR}/lsm6dsv_reg.c")
set_source_files_properties(${MICRAS_LSM6DSV_SOURCES} PROPERTIES COMPILE_OPTIONS "-w")

add_library(micras_firmware STATIC
    ${MICRAS_FIRMWARE_SOURCES}
    ${MICRAS_LSM6DSV_SOURCES}
    ${MICRAS_SIM_HAL_HOST_SOURCES}
    cube/handles.cpp
)

# The fake Cube layer comes first, so <main.h> and friends resolve to it.
target_include_directories(micras_firmware PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/cube"
    "${MICRAS_FIRMWARE_DIR}/include"
    "${MICRAS_FIRMWARE_DIR}/config"
    "${MICRAS_FIRMWARE_DIR}/config/targets/v1"
    "${MICRAS_FIRMWARE_DIR}/micras_core/include"
    "${MICRAS_FIRMWARE_DIR}/micras_nav/include"
    "${MICRAS_FIRMWARE_DIR}/micras_comm/include"
    "${MICRAS_FIRMWARE_DIR}/micras_proxy/include"
    "${MICRAS_FIRMWARE_DIR}/micras_hal/include"
    "${MICRAS_SIM_HAL_HOST_INCLUDE}"
)
target_link_libraries(micras_firmware PUBLIC lsm6dsv_headers)
target_compile_features(micras_firmware PUBLIC cxx_std_23)
set_target_properties(micras_firmware PROPERTIES C_STANDARD 17 C_EXTENSIONS ON)

###############################################################################
## The target
###############################################################################

add_library(micras_sim_micras STATIC
    src/bindings.cpp
    src/micras_target.cpp
    src/pool_variables.cpp
)
target_include_directories(micras_sim_micras PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_include_directories(micras_sim_micras PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/generated")
target_link_libraries(micras_sim_micras PUBLIC micras_firmware micras::hal_host_models micras::sim_app)
target_compile_definitions(micras_sim_micras PRIVATE MICRAS_TARGET_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
add_dependencies(micras_sim_micras micras_firmware_sha)

add_executable(micras_sim src/main.cpp)
target_link_libraries(micras_sim PRIVATE micras_sim_micras)

###############################################################################
## Tests
###############################################################################

if(MICRAS_SIM_TESTS)
    add_subdirectory(tests)
endif()

###############################################################################
## Tools
###############################################################################

add_executable(micras_wall_calibration tools/wall_calibration.cpp)
target_link_libraries(micras_wall_calibration PRIVATE micras_sim_micras)
target_compile_definitions(micras_wall_calibration PRIVATE MICRAS_TARGET_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

add_executable(micras_turn_designer tools/turn_designer.cpp)
target_link_libraries(micras_turn_designer PRIVATE micras_firmware)

add_executable(micras_robot_report tools/robot_report.cpp)
target_link_libraries(micras_robot_report PRIVATE micras_sim_micras)
target_compile_definitions(micras_robot_report PRIVATE MICRAS_TARGET_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

micras_apply_warnings(micras_firmware micras_sim_micras micras_sim micras_wall_calibration micras_turn_designer
    micras_robot_report WERROR ${MICRAS_SIM_WERROR}
)
