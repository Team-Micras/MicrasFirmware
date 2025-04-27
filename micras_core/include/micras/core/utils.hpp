/**
 * @file
 */

#ifndef MICRAS_CORE_UTILS_HPP
#define MICRAS_CORE_UTILS_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>
#include <string_view>

namespace micras::core {
/**
 * @brief Remap a value from one range to another.
 *
 * @tparam T Type of the value.
 * @param value Value to be remapped.
 * @param in_min Minimum value of the input range.
 * @param in_max Maximum value of the input range.
 * @param out_min Minimum value of the output range.
 * @param out_max Maximum value of the output range.
 * @return Remapped value.
 */
template <typename T>
constexpr T remap(T value, T in_min, T in_max, T out_min, T out_max) {
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
 * @brief Move a value towards a target limiting by the step size.
 *
 * @tparam T Type of the value.
 * @param value Current value.
 * @param target Target value.
 * @param step Step size.
 * @return New value.
 */
template <typename T>
constexpr T move_towards(T value, T target, T step) {
    return std::max(std::min(value + step, target), value - step);
}

/**
 * @brief Vary a value from start to end smoothly.
 *
 * @tparam T Type of the value.
 * @param value Current value.
 * @param start Start value.
 * @param end End value.
 * @param resistance Value resistance to variation.
 * @return New value.
 */
template <typename T>
constexpr T transition(T value, T start, T end, T resistance) {
    return end - (end - start) * resistance / (resistance + value * value);
}

/**
 * @brief Create an array of objects calling their constructors from an array of parameters.
 *
 * @tparam T Type of the array.
 * @tparam N Size of the array.
 * @tparam C Type of the parameters.
 * @param parameters List of parameters.
 * @return Array with the objects created from the parameters.
 */
template <typename T, size_t N, typename C>
constexpr std::array<T, N> make_array(const std::array<C, N>& parameters) {
    return [&]<std::size_t... I>(std::index_sequence<I...>) -> std::array<T, N> {
        return {T{parameters[I]}...};
    }(std::make_index_sequence<N>());
}

/**
 * @brief Create an array of objects calling their constructors from a single parameter.
 *
 * @tparam T Type of the array.
 * @tparam N Size of the array.
 * @tparam C Type of the parameter.
 * @param value Parameter.
 * @return Array with the objects created from the parameter.
 */
template <typename T, size_t N, typename C>
constexpr std::array<T, N> make_array(C value) {
    return [&]<std::size_t... I>(std::index_sequence<I...>) -> std::array<T, N> {
        return {(static_cast<void>(I), T{value})...};
    }(std::make_index_sequence<N>());
}

/**
 * @brief Assert an angle to be in the range [-pi, pi].
 *
 * @param angle Angle to be asserted.
 * @return Asserted angle.
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
 * @brief Assert an angle to be in the range [-pi/2, pi/2].
 *
 * @param angle Angle to be asserted.
 * @return Asserted angle.
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
 * @brief Check if two floating point numbers are near each other.
 *
 * @param x First number.
 * @param y Second number.
 * @param tolerance Tolerance.
 * @return True if the numbers are near each other, false otherwise.
 */
constexpr bool is_near(float x, float y, float tolerance = 0.001) {
    return std::abs(x - y) <= tolerance;
}

/**
 * @brief Get the type name of a variable as a string.
 *
 * @cite https://stackoverflow.com/questions/81870/is-it-possible-to-print-a-variables-type-in-standard-c/64490578#64490578
 *
 * @tparam T Type of the variable.
 * @return constexpr auto Type name as a string.
 */
template <typename T>
constexpr auto type_name() {
    std::string_view name, prefix, suffix;
#ifdef __clang__
    name = __PRETTY_FUNCTION__;
    prefix = "auto type_name() [T = ";
    suffix = "]";
#elif defined(__GNUC__)
    name = __PRETTY_FUNCTION__;
    prefix = "constexpr auto type_name() [with T = ";
    suffix = "]";
#elif defined(_MSC_VER)
    name = __FUNCSIG__;
    prefix = "auto __cdecl type_name<";
    suffix = ">(void)";
#endif
    name.remove_prefix(prefix.size());
    name.remove_suffix(suffix.size());
    return name;
}

}  // namespace micras::core

#endif  // MICRAS_CORE_UTILS_HPP
