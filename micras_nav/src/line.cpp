/**
 * @file
 */

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <span>

#include "micras/core/math.hpp"
#include "micras/nav/curve_speed.hpp"
#include "micras/nav/line.hpp"
#include "micras/nav/speed_profile.hpp"
#include "micras/nav/state.hpp"

namespace micras::nav {
bool Line::is_ready() const {
    return this->ready;
}

float Line::length() const {
    return this->size < 2 ? 0.0F : static_cast<float>(this->size - 1) * this->spacing;
}

float Line::duration() const {
    return this->size == 0 ? 0.0F : this->times.at(this->size - 1);
}

float Line::get_finish_time() const {
    return this->finish_time;
}

Pose Line::get_start() const {
    return {.position = {.x = this->xs.at(0), .y = this->ys.at(0)}, .orientation = this->get_heading(0)};
}

SpeedProfile::Sample Line::sample_motion(float time) const {
    return CurveSpeed::sample(
        std::span{this->speeds}.first(this->size), std::span{this->times}.first(this->size), this->spacing, time
    );
}

Line::Point Line::sample_point(float distance) const {
    const float position = std::clamp(distance / this->spacing, 0.0F, static_cast<float>(this->size - 1));
    const auto index = static_cast<uint16_t>(std::min<float>(std::floor(position), static_cast<float>(this->size - 2)));
    const auto next = static_cast<uint16_t>(index + 1);
    const float fraction = position - static_cast<float>(index);

    const float start_heading = this->get_heading(index);
    const float turn = core::math::wrap_angle(this->get_heading(next) - start_heading);

    const float h00 = (1.0F + 2.0F * fraction) * (1.0F - fraction) * (1.0F - fraction);
    const float h10 = fraction * (1.0F - fraction) * (1.0F - fraction) * this->spacing;
    const float h01 = fraction * fraction * (3.0F - 2.0F * fraction);
    const float h11 = fraction * fraction * (fraction - 1.0F) * this->spacing;

    const float start_curvature = this->curvatures.at(index);
    const float end_curvature = this->curvatures.at(next);

    return {
        .pose =
            {
                .position =
                    {
                        .x = h00 * this->xs.at(index) + h10 * std::cos(start_heading) + h01 * this->xs.at(next) +
                             h11 * std::cos(start_heading + turn),
                        .y = h00 * this->ys.at(index) + h10 * std::sin(start_heading) + h01 * this->ys.at(next) +
                             h11 * std::sin(start_heading + turn),
                    },
                .orientation = core::math::wrap_angle(start_heading + fraction * turn),
            },
        .curvature = start_curvature + fraction * (end_curvature - start_curvature),
        .sharpness = (end_curvature - start_curvature) / this->spacing,
    };
}

float Line::get_heading(uint16_t index) const {
    const uint16_t before = index == 0 ? 0 : index - 1;
    const uint16_t after = std::min<uint16_t>(index + 1, this->size - 1);

    return std::atan2(this->ys.at(after) - this->ys.at(before), this->xs.at(after) - this->xs.at(before));
}
}  // namespace micras::nav
