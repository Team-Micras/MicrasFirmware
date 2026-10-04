###############################################################################
## Micras in the simulator: the firmware over micras-lib's host backend
###############################################################################

# Configured by the root's host dispatch (MICRAS_LIB_HAL_BACKEND=host): the application of src/ and
# config/, micras-lib's libraries on its host HAL and chip models, and the simulator's engine.

###############################################################################
## The board: the Micras v1 Cube layer, written by hand
###############################################################################

# Defined before micras-lib, which links it as the HAL's board target. It links the HAL back for the
# host headers and registry, a cycle of two static libraries that CMake links by repeating them.
add_library(micras_host_board STATIC
    cube/handles.cpp
)

target_include_directories(micras_host_board PUBLIC
    cube
)

target_link_libraries(micras_host_board PUBLIC
    micras::hal
)

set(MICRAS_LIB_CUBE_TARGET "micras_host_board" CACHE STRING "Board target micras_hal links, on every backend")

###############################################################################
## micras-lib and the simulator
###############################################################################

add_subdirectory(${PROJECT_SOURCE_DIR}/external/micras-lib ${PROJECT_BINARY_DIR}/external/micras-lib EXCLUDE_FROM_ALL)
add_subdirectory(
    ${PROJECT_SOURCE_DIR}/external/micras-simulation ${PROJECT_BINARY_DIR}/external/micras-simulation EXCLUDE_FROM_ALL
)

include(${PROJECT_SOURCE_DIR}/external/micras-lib/cmake/micras_warnings.cmake)

###############################################################################
## The application, compiled for the host
###############################################################################

include(${PROJECT_SOURCE_DIR}/cmake/app_sources.cmake)

add_library(micras_app STATIC
    ${MICRAS_APP_SOURCES}
    ${MICRAS_APP_CONFIG_SOURCES}
)

target_include_directories(micras_app PUBLIC
    ${MICRAS_APP_INCLUDE_DIRECTORIES}
)

target_link_libraries(micras_app PUBLIC
    micras::nav
    micras::comm
    micras::proxy
)

# The firmware's own main runs on the simulator's firmware thread under another name. Source
# properties belong to the directory that sets them, so this stays next to the target.
set_source_files_properties(${PROJECT_SOURCE_DIR}/src/main.cpp
    PROPERTIES COMPILE_DEFINITIONS "main=micras_firmware_main"
)

###############################################################################
## The target
###############################################################################

# The commit is read at build time, not at configure time, so meta.json never names a firmware the
# binary was not built from.
set(MICRAS_FIRMWARE_SHA_HEADER "${CMAKE_CURRENT_BINARY_DIR}/generated/micras_firmware_sha.hpp")

add_custom_target(micras_firmware_sha
    COMMAND "${CMAKE_COMMAND}" -DFIRMWARE_DIR=${PROJECT_SOURCE_DIR} -DOUTPUT=${MICRAS_FIRMWARE_SHA_HEADER}
            -P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/firmware_sha.cmake"
    BYPRODUCTS "${MICRAS_FIRMWARE_SHA_HEADER}"
    COMMENT "Reading the firmware commit"
    VERBATIM
)

add_library(micras_sim_micras STATIC
    src/bindings.cpp
    src/micras_target.cpp
    src/pool_variables.cpp
)

target_include_directories(micras_sim_micras PUBLIC
    include
)

target_include_directories(micras_sim_micras PRIVATE
    ${CMAKE_CURRENT_BINARY_DIR}/generated
)

target_link_libraries(micras_sim_micras PUBLIC
    micras_app
    micras::proxy_models
    micras::sim_app
)

target_compile_definitions(micras_sim_micras PRIVATE
    MICRAS_TARGET_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)

add_dependencies(micras_sim_micras micras_firmware_sha)

add_executable(micras_sim
    src/main.cpp
)

target_link_libraries(micras_sim PRIVATE
    micras_sim_micras
)

###############################################################################
## Robot tools
###############################################################################

add_executable(micras_wall_calibration
    tools/wall_calibration.cpp
)

target_link_libraries(micras_wall_calibration PRIVATE
    micras_sim_micras
)

target_compile_definitions(micras_wall_calibration PRIVATE
    MICRAS_TARGET_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)

add_executable(micras_robot_report
    tools/robot_report.cpp
)

target_link_libraries(micras_robot_report PRIVATE
    micras_sim_micras
)

target_compile_definitions(micras_robot_report PRIVATE
    MICRAS_TARGET_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)

# The designer reads the robot model and the margins, so it needs neither MuJoCo nor the HAL
add_executable(micras_turn_designer
    tools/turn_designer.cpp
)

target_include_directories(micras_turn_designer PRIVATE
    ${MICRAS_APP_INCLUDE_DIRECTORIES}
)

target_link_libraries(micras_turn_designer PRIVATE
    micras::nav
)

micras_apply_warnings(micras_host_board micras_app micras_sim_micras micras_sim micras_wall_calibration
    micras_robot_report micras_turn_designer WERROR ${MICRAS_WERROR}
)

###############################################################################
## Tests
###############################################################################

add_subdirectory(tests)

###############################################################################
## Lint: the sources of sim/, over this build's compilation database
###############################################################################

# The robot build formats them with the rest of the firmware and lints what it compiles for the ARM core
include(${PROJECT_SOURCE_DIR}/cmake/style.cmake)

file(GLOB_RECURSE SIM_LINT_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
generate_lint_target(SIM_LINT_SOURCES)

###############################################################################
## Recipes: the runs, the contest, the baseline and the robot tools
###############################################################################

# Each calls a script of scripts/ with every path as an argument; runs land in runs/ of this build
# directory. MICRAS_SIM_EXACT is the simulator's option.
set(MICRAS_SIM_BASELINE "v3" CACHE STRING "Baseline version sim_compare_baseline and sim_record_baseline use")
set(MICRAS_SIM_CONTEST_MAZES
    "maze1;maze2;apec2016;apec2017;apec2018;apec2019;japan2013ef;japan2017ef;uk2016f;alljapan-033-2012-exp-fin"
    CACHE STRING "The mazes sim_contest and sim_contest_all run in"
)

set(MICRAS_SIM_SCRIPTS "${CMAKE_CURRENT_SOURCE_DIR}/scripts")
set(MICRAS_SIM_SCENARIOS "${CMAKE_CURRENT_SOURCE_DIR}/scenarios")
set(MICRAS_SIM_RUNS "${CMAKE_CURRENT_BINARY_DIR}/runs")
set(MICRAS_SIM_PLUGIN "${CMAKE_CURRENT_SOURCE_DIR}/tools/analysis.py")
set(MICRAS_SIM_BASELINE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/baselines/${MICRAS_SIM_BASELINE}")
set(MICRAS_SIM_CHECKED_RUNS idle explore)
set(MICRAS_SIM_EXPLORE_SECONDS 30)

set(MICRAS_SIM_RUN_IDLE
    "${MICRAS_SIM_SCRIPTS}/run_idle.sh" "$<TARGET_FILE:micras_sim>" "${MICRAS_SIM_SCENARIOS}" "${MICRAS_SIM_RUNS}"
)
set(MICRAS_SIM_RUN_EXPLORE
    "${MICRAS_SIM_SCRIPTS}/run_explore.sh" "$<TARGET_FILE:micras_sim>" "${MICRAS_SIM_SCENARIOS}"
    "${MICRAS_SIM_RUNS}" "${MICRAS_SIM_EXPLORE_SECONDS}"
)
set(MICRAS_SIM_COMPARE_BASELINE
    "${MICRAS_SIM_SCRIPTS}/compare_baseline.sh" "${MICRAS_SIM_TOOLS_DIR}" "${MICRAS_SIM_PLUGIN}" "${MICRAS_SIM_RUNS}"
    "${MICRAS_SIM_BASELINE_DIR}" "${MICRAS_SIM_EXACT}" ${MICRAS_SIM_CHECKED_RUNS}
)
set(MICRAS_SIM_CHECK_FLASH
    "${MICRAS_SIM_SCRIPTS}/check_flash.sh" "$<TARGET_FILE:micras_sim>" "${MICRAS_SIM_SCENARIOS}" "${MICRAS_SIM_RUNS}"
    "${MICRAS_SIM_TOOLS_DIR}"
)

add_custom_target(sim_run_idle
    COMMAND ${MICRAS_SIM_RUN_IDLE}
    COMMENT "The robot left alone for 4 s -> runs/idle"
    VERBATIM USES_TERMINAL
)

add_custom_target(sim_run_explore
    COMMAND ${MICRAS_SIM_RUN_EXPLORE}
    COMMENT "The first ${MICRAS_SIM_EXPLORE_SECONDS} s of an exploration, with its flash -> runs/explore"
    VERBATIM USES_TERMINAL
)

foreach(CONTEST explore_solve explore_solve_all)
    if(CONTEST STREQUAL "explore_solve")
        set(CONTEST_TARGET sim_contest)
    else()
        set(CONTEST_TARGET sim_contest_all)
    endif()

    add_custom_target(${CONTEST_TARGET}
        COMMAND "${MICRAS_SIM_SCRIPTS}/contest.sh" "$<TARGET_FILE:micras_sim>" "${MICRAS_SIM_SCENARIOS}/${CONTEST}.toml"
                "${MICRAS_SIM_RUNS}/contest/${CONTEST}" "${MICRAS_SIM_TOOLS_DIR}" ${MICRAS_SIM_CONTEST_MAZES}
        COMMENT "The contest (${CONTEST}) in ${MICRAS_SIM_CONTEST_MAZES}"
        VERBATIM USES_TERMINAL
    )
endforeach()

add_custom_target(sim_compare_baseline
    COMMAND ${MICRAS_SIM_COMPARE_BASELINE}
    COMMENT "Comparing runs/idle and runs/explore with baselines/${MICRAS_SIM_BASELINE}"
    VERBATIM USES_TERMINAL
)

add_custom_target(sim_record_baseline
    COMMAND "${CMAKE_CTEST_COMMAND}" --test-dir "${CMAKE_CURRENT_BINARY_DIR}" --output-on-failure
    COMMAND ${MICRAS_SIM_RUN_IDLE}
    COMMAND ${MICRAS_SIM_RUN_EXPLORE}
    COMMAND "${MICRAS_SIM_SCRIPTS}/record_baseline.sh" "${MICRAS_SIM_TOOLS_DIR}" "${MICRAS_SIM_PLUGIN}"
            "${MICRAS_SIM_RUNS}" "${MICRAS_SIM_BASELINE_DIR}" ${MICRAS_SIM_CHECKED_RUNS}
    COMMENT "Recording baselines/${MICRAS_SIM_BASELINE}"
    VERBATIM USES_TERMINAL
)

add_custom_target(sim_check_flash
    COMMAND ${MICRAS_SIM_RUN_EXPLORE}
    COMMAND ${MICRAS_SIM_CHECK_FLASH}
    COMMENT "Checking that the flash outlives a run"
    VERBATIM USES_TERMINAL
)

add_custom_target(sim_check
    COMMAND "${CMAKE_CTEST_COMMAND}" --test-dir "${CMAKE_CURRENT_BINARY_DIR}" --output-on-failure --parallel
    COMMAND ${MICRAS_SIM_RUN_EXPLORE}
    COMMAND ${MICRAS_SIM_CHECK_FLASH}
    COMMAND ${MICRAS_SIM_RUN_IDLE}
    COMMAND ${MICRAS_SIM_COMPARE_BASELINE}
    COMMAND "${MICRAS_SIM_SCRIPTS}/check_health.sh" "${MICRAS_SIM_TOOLS_DIR}" "${MICRAS_SIM_RUNS}"
            ${MICRAS_SIM_CHECKED_RUNS}
    COMMENT "The Micras gate: tests, flash, the checked runs, their baseline and their health"
    VERBATIM USES_TERMINAL
)

add_custom_target(sim_turn_designs
    COMMAND "${MICRAS_SIM_SCRIPTS}/turn_designs.sh" "$<TARGET_FILE:micras_turn_designer>"
            "${PROJECT_SOURCE_DIR}/config/two_bend_turns.hpp"
    COMMENT "Designing the turns of two bends into the firmware's config/two_bend_turns.hpp"
    VERBATIM USES_TERMINAL
)

add_custom_target(sim_robot_report
    COMMAND "$<TARGET_FILE:micras_robot_report>"
    COMMENT "robot.toml against the firmware's robot.hpp"
    VERBATIM USES_TERMINAL
)

add_custom_target(sim_wall_calibration
    COMMAND "$<TARGET_FILE:micras_wall_calibration>"
    COMMENT "Each wall sensor's gain for robot.toml"
    VERBATIM USES_TERMINAL
)

add_custom_target(sim_serve
    COMMAND "${MICRAS_SIM_SCRIPTS}/serve.sh" "$<TARGET_FILE:micras_sim>" "${MICRAS_SIM_SCENARIOS}" "${MICRAS_SIM_RUNS}"
    COMMENT "An exploration open to micras-monitor on ws://localhost:8080"
    VERBATIM USES_TERMINAL
)

add_custom_target(sim_watch
    COMMAND "${MICRAS_SIM_SCRIPTS}/watch.sh" "$<TARGET_FILE:micras_sim>" "${MICRAS_SIM_SCENARIOS}" "${MICRAS_SIM_RUNS}"
    COMMENT "An exploration in a window"
    VERBATIM USES_TERMINAL
)

foreach(RECIPE sim_run_idle sim_run_explore sim_contest sim_contest_all sim_record_baseline sim_check_flash sim_check
               sim_serve sim_watch)
    add_dependencies(${RECIPE} micras_sim)
endforeach()

add_dependencies(sim_check micras_sim_tests)
add_dependencies(sim_record_baseline micras_sim_tests)

add_dependencies(sim_turn_designs micras_turn_designer)
add_dependencies(sim_robot_report micras_robot_report)
add_dependencies(sim_wall_calibration micras_wall_calibration)
