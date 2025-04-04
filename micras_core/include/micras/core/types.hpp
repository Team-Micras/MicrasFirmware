/**
 * @file
 */

#ifndef MICRAS_CORE_TYPES_HPP
#define MICRAS_CORE_TYPES_HPP

#include <cstdint>

namespace micras::core {
/**
 * @brief Possible values for the observation of a wall.
 */
enum Observation : uint8_t {
    UNKNOWN = 0,
    FREE_SPACE = 1,
    WALL = 2
};

/**
 * @brief Types of wall following the robot is able to do.
 */
enum FollowWallType : uint8_t {
    NONE = 0,
    FRONT = 1,
    LEFT = 2,
    RIGHT = 3,
    PARALLEL = 4,
    BACK = 5,
};

/**
 * @brief Possible objectives of the robot.
 */
enum Objective : uint8_t {
    EXPLORE = 0,
    RETURN = 1,
    SOLVE = 2
};
}  // namespace micras::core

#endif  // MICRAS_CORE_TYPES_HPP
