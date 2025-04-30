/**
 * @file
 */

#ifndef MICRAS_CORE_TYPES_HPP
#define MICRAS_CORE_TYPES_HPP

#include <cstdint>

namespace micras::core {
/**
 * @brief Possible values for the observation from the sensors.
 */
struct Observation {
    bool left{};
    bool front{};
    bool right{};
};

/**
 * @brief Possible objectives of the robot.
 */
enum Objective : uint8_t {
    EXPLORE = 0,
    RETURN = 1,
    SOLVE = 2
};

/**
 * @brief Struct to store the index of each of the wall sensors.
 */
struct WallSensorsIndex {
    uint8_t left_front{};
    uint8_t left{};
    uint8_t right{};
    uint8_t right_front{};
};
}  // namespace micras::core

#endif  // MICRAS_CORE_TYPES_HPP
