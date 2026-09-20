/**
 * @file
 */

#ifndef MICRAS_CORE_MATH_HPP
#define MICRAS_CORE_MATH_HPP

#include <concepts>
#include <cstdint>
#include <numbers>

/**
 * @brief Elementary functions that are constant expressions on every compiler.
 *
 * @note The functions of the standard library only become constant expressions in C++26, and only
 * GCC accepts them earlier as an extension. Everything computed when the firmware is compiled, such
 * as the shape of each turn, goes through these instead, so that the same sources build with clang
 * and with the compilers of the simulators. They are also usable at run time, where the series are
 * short enough to be cheaper than a table.
 */
namespace micras::core::math {
/**
 * @brief Absolute value.
 *
 * @tparam T Floating point type.
 * @param value Value.
 * @return The absolute value.
 */
template <std::floating_point T>
constexpr T abs(T value) {
    return value < T{0} ? -value : value;
}

/**
 * @brief Square root by Newton iterations.
 *
 * @tparam T Floating point type.
 * @param value Non negative value.
 * @return The square root of the value, or zero for a non positive value.
 */
template <std::floating_point T>
constexpr T sqrt(T value) {
    if (value <= T{0}) {
        return T{0};
    }

    T estimate = value > T{1} ? value : T{1};

    for (uint8_t i = 0; i < 64; i++) {
        const T next = (estimate + value / estimate) / T{2};

        if (abs(next - estimate) <= estimate * T{1e-16}) {
            return next;
        }

        estimate = next;
    }

    return estimate;
}

/**
 * @brief Wrap an angle to the range [-pi, pi].
 *
 * @tparam T Floating point type.
 * @param angle Angle in radians.
 * @return The equivalent angle in [-pi, pi].
 */
template <std::floating_point T>
constexpr T wrap_angle(T angle) {
    constexpr T two_pi = T{2} * std::numbers::pi_v<T>;

    const auto turns = static_cast<int64_t>(angle / two_pi + (angle < T{0} ? T{-0.5} : T{0.5}));

    return angle - static_cast<T>(turns) * two_pi;
}

/**
 * @brief Sine by its Taylor series, after folding the angle into [-pi/2, pi/2].
 *
 * @tparam T Floating point type.
 * @param angle Angle in radians.
 * @return The sine of the angle.
 */
template <std::floating_point T>
constexpr T sin(T angle) {
    angle = wrap_angle(angle);

    if (angle > std::numbers::pi_v<T> / T{2}) {
        angle = std::numbers::pi_v<T> - angle;
    } else if (angle < -std::numbers::pi_v<T> / T{2}) {
        angle = -std::numbers::pi_v<T> - angle;
    }

    const T squared = angle * angle;
    T       term = angle;
    T       sum = angle;

    for (uint8_t order = 1; order < 12; order++) {
        term *= -squared / static_cast<T>((2 * order) * (2 * order + 1));
        sum += term;
    }

    return sum;
}

/**
 * @brief Cosine, as the sine of the complementary angle.
 *
 * @tparam T Floating point type.
 * @param angle Angle in radians.
 * @return The cosine of the angle.
 */
template <std::floating_point T>
constexpr T cos(T angle) {
    return sin(angle + std::numbers::pi_v<T> / T{2});
}

/**
 * @brief Point reached by a clothoid that starts straight along the x axis.
 *
 * @tparam T Floating point type.
 */
template <std::floating_point T>
struct ClothoidPoint {
    T x;
    T y;
};

/**
 * @brief Evaluate a clothoid, the curve whose curvature grows linearly with the distance traveled.
 *
 * @details The path of a robot that moves at a constant speed while its angular speed ramps up is a
 * clothoid. With a heading of `sharpness * s^2 / 2` after a distance `s`, its coordinates are the
 * Fresnel integrals `x = int cos(sharpness * t^2 / 2) dt` and `y = int sin(sharpness * t^2 / 2) dt`.
 * They are evaluated here by their power series in the heading, which converges for every argument
 * and needs about ten terms for the quarter turn that is the most a ramp of a slalom turn covers.
 *
 * @tparam T Floating point type.
 * @param distance Distance traveled along the curve, in meters.
 * @param sharpness Rate of change of the curvature with the distance, in 1/m^2.
 * @param terms Number of terms of the series.
 * @return The coordinates of the point, in the frame of the start of the curve.
 */
template <std::floating_point T>
constexpr ClothoidPoint<T> clothoid(T distance, T sharpness, uint8_t terms = 14) {
    const T heading = sharpness * distance * distance / T{2};
    const T squared = heading * heading;

    T even_term = T{1};
    T odd_term = heading;
    T x_sum = T{0};
    T y_sum = T{0};

    for (uint8_t order = 0; order < terms; order++) {
        x_sum += even_term / static_cast<T>(4 * order + 1);
        y_sum += odd_term / static_cast<T>(4 * order + 3);

        even_term *= -squared / static_cast<T>((2 * order + 1) * (2 * order + 2));
        odd_term *= -squared / static_cast<T>((2 * order + 2) * (2 * order + 3));
    }

    return {.x = distance * x_sum, .y = distance * y_sum};
}
}  // namespace micras::core::math

#endif  // MICRAS_CORE_MATH_HPP
