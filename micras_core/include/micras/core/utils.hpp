/**
 * @file utils.hpp
 *
 * @brief Core Utils functions
 *
 * @date 09/2024
 */

#ifndef MICRAS_CORE_UTILS_HPP
#define MICRAS_CORE_UTILS_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>

namespace micras::core {
/**
 * @brief Remap a value from one range to another
 *
 * @tparam T Type of the value
 * @param value Value to be remapped
 * @param in_min Minimum value of the input range
 * @param in_max Maximum value of the input range
 * @param out_min Minimum value of the output range
 * @param out_max Maximum value of the output range
 * @return T Remapped value
 */
template <typename T>
constexpr T remap(T value, T in_min, T in_max, T out_min, T out_max) {
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
 * @brief Move a value towards a target limiting the step size
 *
 * @tparam T Type of the value
 * @param value Current value
 * @param target Target value
 * @param step Step size
 * @return T New value
 */
template <typename T>
constexpr T move_towards(T value, T target, T step) {
    return std::max(std::min(value + step, target), value - step);
}

/**
 * @brief Decay a value towards zero with a damping factor
 *
 * @tparam T Type of the value
 * @param value Current value
 * @param damping Damping factor
 * @return T New value
 */
template <typename T>
constexpr T decay(T value, T damping) {
    return damping / (damping + value * value);
}

/**
 * @brief Create an array objects calling its constructors from an array of parameters
 *
 * @tparam T Type of the array
 * @tparam N Size of the array
 * @tparam C Type of the parameters
 * @param parameters List of parameters
 * @return std::array<T, N> Array with the objects created from the parameters
 */
template <typename T, size_t N, typename C>
constexpr std::array<T, N> make_array(const std::array<C, N>& parameters) {
    return [&]<std::size_t... I>(std::index_sequence<I...>) -> std::array<T, N> {
        return {T{parameters[I]}...};
    }(std::make_index_sequence<N>());
}

/**
 * @brief Create an array objects calling its constructors from a single parameter
 *
 * @tparam T Type of the array
 * @tparam N Size of the array
 * @tparam C Type of the parameter
 * @param value Parameter
 * @return std::array<T, N> Array with the objects created from the parameter
 */
template <typename T, size_t N, typename C>
constexpr std::array<T, N> make_array(C value) {
    return [&]<std::size_t... I>(std::index_sequence<I...>) -> std::array<T, N> {
        return {(static_cast<void>(I), T{value})...};
    }(std::make_index_sequence<N>());
}

/**
 * @brief Assert an angle to be in the range [-pi, pi]
 *
 * @param angle Angle to be asserted
 * @return float Asserted angle
 */
constexpr float assert_angle(float angle) {
    angle = std::fmod(angle, 2 * std::numbers::pi_v<float>);

    if (angle > std::numbers::pi_v<float>) {
        angle -= 2 * std::numbers::pi_v<float>;
    } else if (angle < -std::numbers::pi_v<float>) {
        angle += 2 * std::numbers::pi_v<float>;
    }

    return angle;
}

/**
 * @brief Assert an angle to be in the range [-pi/2, pi/2]
 *
 * @param angle Angle to be asserted
 * @return float Asserted angle
 */
constexpr float assert_half_angle(float angle) {
    angle = std::fmod(angle, std::numbers::pi_v<float>);

    if (angle > std::numbers::pi_v<float> / 2.0F) {
        angle -= std::numbers::pi_v<float>;
    } else if (angle < -std::numbers::pi_v<float> / 2.0F) {
        angle += std::numbers::pi_v<float>;
    }

    return angle;
}

/**
 * @brief Check if two floating point numbers are near each other
 *
 * @param a First number
 * @param b Second number
 * @param tolerance Tolerance
 * @return true If the numbers are near each other, false otherwise
 */
constexpr bool is_near(float a, float b, float tolerance = 0.001) {
    return std::abs(a - b) < tolerance;
}
}  // namespace micras::core

#endif  // MICRAS_CORE_UTILS_HPP
