/**
 * @file
 */

#ifndef MICRAS_TARGET_HPP
#define MICRAS_TARGET_HPP

/**
 * @brief Board specific configuration, selected at configure time by BOARD_VERSION.
 *
 * @note Everything that names a pin, a peripheral handle or a component of the board lives in
 * targets/<board>.hpp, so that supporting another board, or reusing the packages in another
 * project, is a matter of adding one file there. MICRAS_TARGET_HEADER is defined by CMake from
 * BOARD_VERSION, the same variable that selects the STM32CubeMX project.
 */
#ifndef MICRAS_TARGET_HEADER
    #error "MICRAS_TARGET_HEADER is not defined, the build system did not select a board"
#endif

#include MICRAS_TARGET_HEADER  // IWYU pragma: export

#endif  //  MICRAS_TARGET_HPP
