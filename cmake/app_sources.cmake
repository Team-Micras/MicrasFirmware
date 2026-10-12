###############################################################################
## Application sources
###############################################################################

# The one list of the application's sources and include directories, shared by the robot image and
# the simulation. The globs return sorted absolute paths, and that order is the order of the objects
# on the link line, which the image depends on.
get_filename_component(MICRAS_APP_ROOT ${CMAKE_CURRENT_LIST_DIR}/.. ABSOLUTE)

# The main loop, the state machine and the interface, main.cpp included
file(GLOB_RECURSE MICRAS_APP_SOURCES CONFIGURE_DEPENDS
    ${MICRAS_APP_ROOT}/src/*.cpp
)

# The configuration computed when the firmware is compiled
file(GLOB MICRAS_APP_CONFIG_SOURCES CONFIGURE_DEPENDS
    ${MICRAS_APP_ROOT}/config/*.cpp
)

# The board directory goes on the include path, so that "target.hpp" resolves to it and nothing
# has to name the board. The headers the build generates, the turns of two bends, are in the build
# directory.
set(MICRAS_APP_GENERATED_DIRECTORY ${CMAKE_BINARY_DIR}/generated)

# The maze the robot runs in, from config/mazes: its size, start, goal, cell size and wall thickness.
# The simulation and its baselines run the classic one.
set(MICRAS_MAZE "classic" CACHE STRING "Maze the firmware is built for, a directory of config/mazes")

if(NOT EXISTS ${MICRAS_APP_ROOT}/config/mazes/${MICRAS_MAZE}/maze_config.hpp)
    message(FATAL_ERROR "No maze at config/mazes/${MICRAS_MAZE}/maze_config.hpp for MICRAS_MAZE=${MICRAS_MAZE}")
endif()

set(MICRAS_APP_INCLUDE_DIRECTORIES
    ${MICRAS_APP_ROOT}/include
    ${MICRAS_APP_ROOT}/config
    ${MICRAS_APP_ROOT}/config/targets/${BOARD_VERSION}
    ${MICRAS_APP_ROOT}/config/mazes/${MICRAS_MAZE}
    ${MICRAS_APP_GENERATED_DIRECTORY}
)
