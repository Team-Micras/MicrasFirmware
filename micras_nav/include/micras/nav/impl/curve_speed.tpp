/**
 * @file
 */

#ifndef MICRAS_NAV_CURVE_SPEED_TPP
#define MICRAS_NAV_CURVE_SPEED_TPP

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <span>

#include "micras/nav/motion_limits.hpp"
#include "micras/nav/turn_table.hpp"

namespace micras::nav {
template <typename F>
CurveSpeed CurveSpeed::braking(
    F&& bending, float start_distance, float end_distance, float start_speed, const CurveLimits& limits
) {
    CurveSpeed motion;
    motion.start_distance = start_distance;

    const float available = std::max(end_distance - start_distance, 0.0F);
    float       spacing = turn_spacing;
    bool        ended = false;

    for (uint8_t doublings = 0; doublings < max_spacing_doublings and not ended; doublings++) {
        ended = motion.plan_braking(bending, available, start_speed, std::ceil(available / spacing), limits);
        spacing *= 2.0F;
    }

    if (not ended) {
        motion.plan_braking(bending, available, start_speed, max_intervals, limits);
    }

    const std::size_t samples = motion.intervals + std::size_t{1};

    integrate(std::span{motion.speeds}.first(samples), motion.spacing, std::span{motion.times}.first(samples));

    return motion;
}

template <typename F>
bool CurveSpeed::plan_braking(
    F& bending, float available, float start_speed, float intervals_to_end, const CurveLimits& limits
) {
    const float wanted = std::max(intervals_to_end, 1.0F);
    const float last = std::min(wanted, static_cast<float>(max_intervals));

    this->spacing = available / wanted;
    this->speeds.front() = start_speed;

    uint8_t steps = 0;

    while (static_cast<float>(steps) < last and this->speeds.at(steps) > 0.0F) {
        const float   speed = this->speeds.at(steps);
        const float   distance = this->start_distance + static_cast<float>(steps) * this->spacing;
        const Bending start = bending(distance);
        const Bending end = bending(distance + this->spacing);
        const float   deceleration = limits.get_deceleration(
            speed, {
                       .curvature = std::max(std::abs(start.curvature), std::abs(end.curvature)),
                       .sharpness = std::max(std::abs(start.sharpness), std::abs(end.sharpness)),
                   }
        );

        this->speeds.at(steps + 1) = std::sqrt(std::max(speed * speed - 2.0F * this->spacing * deceleration, 0.0F));
        steps++;
    }

    this->intervals = steps;

    return this->speeds.at(steps) <= 0.0F or static_cast<float>(steps) >= wanted;
}

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
