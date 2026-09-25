/**
 * @file
 */

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <span>

#include "micras/nav/curve_speed.hpp"
#include "micras/nav/motion_limits.hpp"
#include "micras/nav/speed_profile.hpp"
#include "micras/nav/turn_table.hpp"

namespace micras::nav {
namespace {
/**
 * @brief Bending of a turn at its samples, as the passes of CurveSpeed read it.
 */
struct TurnBending {
    /**
     * @brief Get the curvature at a sample and the sharpness between it and the next one.
     *
     * @param index The index of the sample.
     * @return The bending.
     */
    Bending operator()(std::size_t index) const {
        const auto  position = static_cast<float>(index);
        const float middle = std::min(position + 0.5F, static_cast<float>(this->intervals) - 0.5F);

        return {
            .curvature = this->shape.bending_at(position * this->spacing).curvature,
            .sharpness = this->shape.bending_at(middle * this->spacing).sharpness,
        };
    }

    // NOLINTNEXTLINE(*-avoid-const-or-ref-data-members) a view that lives for one call
    const TurnShape& shape;
    uint8_t          intervals;
    float            spacing;
};
}  // namespace

CurveSpeed::CurveSpeed(const TurnShape& shape, float start_speed, float end_speed, const CurveLimits& limits) :
    intervals{get_intervals(shape)}, spacing{shape.length() / static_cast<float>(get_intervals(shape))} {
    const std::span<float> speeds = std::span{this->speeds}.first(this->intervals + 1U);

    plan(
        TurnBending{.shape = shape, .intervals = this->intervals, .spacing = this->spacing}, speeds, this->spacing,
        start_speed, end_speed, limits
    );
    integrate(speeds, this->spacing, std::span{this->times}.first(this->intervals + 1U));
}

float CurveSpeed::duration() const {
    return this->times.at(this->intervals);
}

SpeedProfile::Sample CurveSpeed::sample(float time) const {
    return sample(
        std::span{this->speeds}.first(this->intervals + 1U), std::span{this->times}.first(this->intervals + 1U),
        this->spacing, time
    );
}

float CurveSpeed::get_entry_speed(const TurnShape& shape, float end_speed, const CurveLimits& limits) {
    const uint8_t     intervals = get_intervals(shape);
    const float       spacing = shape.length() / static_cast<float>(intervals);
    const TurnBending bending{.shape = shape, .intervals = intervals, .spacing = spacing};

    Bending after = bending(intervals);
    float   speed = std::min(end_speed, get_limit(bending(intervals - 1U), after, limits));

    for (uint8_t i = intervals; i > 0; i--) {
        const Bending at = bending(i - 1U);
        const Bending before = bending(i > 1 ? i - 2U : 0U);
        const float   deceleration =
            limits.get_deceleration(speed, {.curvature = after.curvature, .sharpness = at.sharpness});

        speed = std::min(std::sqrt(speed * speed + 2.0F * spacing * deceleration), get_limit(before, at, limits));
        after = at;
    }

    return speed;
}

float CurveSpeed::get_exit_speed(const TurnShape& shape, float start_speed, const CurveLimits& limits) {
    const uint8_t     intervals = get_intervals(shape);
    const float       spacing = shape.length() / static_cast<float>(intervals);
    const TurnBending bending{.shape = shape, .intervals = intervals, .spacing = spacing};

    Bending at = bending(0);
    float   speed = std::min(start_speed, get_limit(at, at, limits));

    for (uint8_t i = 0; i < intervals; i++) {
        const Bending next = bending(i + 1U);
        const float   acceleration = limits.get_acceleration(speed, at);

        speed = std::min(std::sqrt(speed * speed + 2.0F * spacing * acceleration), get_limit(at, next, limits));
        at = next;
    }

    return speed;
}

void CurveSpeed::integrate(std::span<const float> speeds, float spacing, std::span<float> times) {
    if (times.empty()) {
        return;
    }

    times[0] = 0.0F;

    for (std::size_t i = 1; i < times.size(); i++) {
        times[i] = times[i - 1] + 2.0F * spacing / std::max(speeds[i - 1] + speeds[i], min_speed_sum);
    }
}

SpeedProfile::Sample
    CurveSpeed::sample(std::span<const float> speeds, std::span<const float> times, float spacing, float time) {
    if (times.size() < 2) {
        return {.distance = 0.0F, .speed = speeds.empty() ? 0.0F : speeds.front(), .acceleration = 0.0F};
    }

    time = std::clamp(time, 0.0F, times.back());

    const auto  next = std::ranges::upper_bound(times, time);
    std::size_t index = next == times.begin() ? 0 : static_cast<std::size_t>(next - times.begin()) - 1;
    index = std::min(index, times.size() - 2);

    const float start = speeds[index];
    const float end = speeds[index + 1];
    const float acceleration = (end * end - start * start) / (2.0F * spacing);
    const float elapsed = std::min(time - times[index], times[index + 1] - times[index]);

    return {
        .distance = static_cast<float>(index) * spacing +
                    std::min(start * elapsed + acceleration * elapsed * elapsed / 2.0F, spacing),
        .speed = start + acceleration * elapsed,
        .acceleration = acceleration,
    };
}

uint8_t CurveSpeed::get_intervals(const TurnShape& shape) {
    const auto wanted = static_cast<int32_t>(std::ceil(shape.length() / turn_spacing));
    return static_cast<uint8_t>(std::clamp<int32_t>(wanted, 1, max_intervals));
}

float CurveSpeed::get_limit(const Bending& before, const Bending& at, const CurveLimits& limits) {
    return limits.get_speed_limit(
        {.curvature = at.curvature, .sharpness = std::max(std::abs(before.sharpness), std::abs(at.sharpness))}
    );
}
}  // namespace micras::nav
