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
## The firmware, compiled for the host
###############################################################################

# Everything but micras_hal, which hal_host replaces, and the proxies of the two SPI
# chips, which proxy/ replaces. FmacFilter is unused and needs the FMAC. The firmware's
# own main is compiled under another name, and the simulator runs it on the firmware
# thread.
file(GLOB MICRAS_FIRMWARE_SOURCES CONFIGURE_DEPENDS
    "${MICRAS_FIRMWARE_DIR}/micras_core/src/*.cpp"
    "${MICRAS_FIRMWARE_DIR}/micras_nav/src/*.cpp"
    "${MICRAS_FIRMWARE_DIR}/micras_comm/src/*.cpp"
    "${MICRAS_FIRMWARE_DIR}/micras_proxy/src/*.cpp"
    "${MICRAS_FIRMWARE_DIR}/src/*.cpp"
    "${MICRAS_FIRMWARE_DIR}/src/states/*.cpp"
)
list(REMOVE_ITEM MICRAS_FIRMWARE_SOURCES
    "${MICRAS_FIRMWARE_DIR}/micras_proxy/src/imu.cpp"
    "${MICRAS_FIRMWARE_DIR}/micras_proxy/src/rotary_sensor.cpp"
    "${MICRAS_FIRMWARE_DIR}/micras_proxy/src/fmac_filter.cpp"
)
set_source_files_properties("${MICRAS_FIRMWARE_DIR}/src/main.cpp"
    PROPERTIES COMPILE_DEFINITIONS "main=micras_firmware_main"
)

add_library(micras_firmware STATIC
    ${MICRAS_FIRMWARE_SOURCES}
    ${MICRAS_SIM_HAL_HOST_SOURCES}
    cube/handles.cpp
    proxy/imu.cpp
    proxy/rotary_sensor.cpp
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
target_link_libraries(micras_sim_micras PUBLIC micras_firmware micras::sim_app)
target_compile_definitions(micras_sim_micras PRIVATE MICRAS_TARGET_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_options(micras_sim_micras PRIVATE -Wall -Wextra)
add_dependencies(micras_sim_micras micras_firmware_sha)

add_executable(micras_sim src/main.cpp)
target_link_libraries(micras_sim PRIVATE micras_sim_micras)
target_compile_options(micras_sim PRIVATE -Wall -Wextra)

###############################################################################
## Tests
###############################################################################

if(MICRAS_TESTS)
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
