/**
 * @file
 */

#include <cmath>

#include "micras/nav/motion_limits.hpp"
#include "micras/nav/speed_ramp.hpp"
#include "micras/nav/state.hpp"

namespace micras::nav {
void SpeedRamp::start(const Twist& speed, const MotionLimits& linear, const MotionLimits& angular) {
    this->speed = speed;
    this->linear = linear;
    this->angular = angular;
}

SpeedRamp::Sample SpeedRamp::update(float elapsed_time) {
    const float linear_acceleration = slow_down(this->speed.linear, this->linear, elapsed_time);
    const float angular_acceleration = slow_down(this->speed.angular, this->angular, elapsed_time);

    return {.twist = this->speed, .acceleration = {.linear = linear_acceleration, .angular = angular_acceleration}};
}

bool SpeedRamp::is_finished() const {
    return this->speed.linear == 0.0F and this->speed.angular == 0.0F;
}

float SpeedRamp::slow_down(float& speed, const MotionLimits& limits, float elapsed_time) {
    const float deceleration = limits.deceleration_at(std::abs(speed));

    if (std::abs(speed) <= deceleration * elapsed_time) {
        const float acceleration = -speed / elapsed_time;
        speed = 0.0F;
        return acceleration;
    }

    const float acceleration = -std::copysign(deceleration, speed);
    speed += acceleration * elapsed_time;
    return acceleration;
}
}  // namespace micras::nav
