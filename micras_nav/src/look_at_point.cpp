/**
 * @file look_at_point.cpp
 *
 * @brief Nav LookAtPoint class implementation
 *
 * @date 10/2024
 */

#include "micras/core/utils.hpp"
#include "micras/nav/look_at_point.hpp"

namespace micras::nav {
LookAtPoint::LookAtPoint(Config config) : pid(config.pid), tolerance{config.tolerance} { }

Twist LookAtPoint::action(const Pose& pose, const Point& goal) {
    float angular_target = pose.position.angle_between(goal);
    return {0.0F, this->pid.update(core::assert_angle(pose.orientation - angular_target))};
}

bool LookAtPoint::finished(const Pose& pose, const Point& goal) const {
    return std::abs(core::assert_angle(pose.orientation - pose.position.angle_between(goal))) <= this->tolerance;
}
}  // namespace micras::nav
