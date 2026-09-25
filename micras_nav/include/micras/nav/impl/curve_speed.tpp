/**
 * @file
 */

#ifndef MICRAS_NAV_CURVE_SPEED_TPP
#define MICRAS_NAV_CURVE_SPEED_TPP

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <span>

#include "micras/nav/curve_speed.hpp"
#include "micras/nav/motion_limits.hpp"
#include "micras/nav/turn_table.hpp"

namespace micras::nav {
template <typename F>
void CurveSpeed::plan(
    F&& bending, std::span<float> speeds, float spacing, float start_speed, float end_speed, const CurveLimits& limits
) {
    if (speeds.empty()) {
        return;
    }

    const std::size_t last = speeds.size() - 1;

    Bending before = bending(0);

    for (std::size_t i = 0; i <= last; i++) {
        const Bending at = bending(i);
        speeds[i] = get_limit(before, at, limits);
        before = at;
    }

    speeds[0] = std::min(speeds[0], start_speed);

    for (std::size_t i = 0; i < last; i++) {
        const float acceleration = limits.get_acceleration(speeds[i], bending(i));
        speeds[i + 1] = std::min(speeds[i + 1], std::sqrt(speeds[i] * speeds[i] + 2.0F * spacing * acceleration));
    }

    speeds[last] = std::min(speeds[last], end_speed);

    for (std::size_t i = last; i > 0; i--) {
        const Bending at{.curvature = bending(i).curvature, .sharpness = bending(i - 1).sharpness};
        const float   deceleration = limits.get_deceleration(speeds[i], at);
        speeds[i - 1] = std::min(speeds[i - 1], std::sqrt(speeds[i] * speeds[i] + 2.0F * spacing * deceleration));
    }
}
}  // namespace micras::nav

#endif  // MICRAS_NAV_CURVE_SPEED_TPP
