/**
 * @file
 */

#include <algorithm>
#include <cmath>

#include "micras/nav/lattice.hpp"
#include "micras/nav/motion_limits.hpp"
#include "micras/nav/turn_table.hpp"

namespace micras::nav {
float MotionLimits::acceleration_at(float speed) const {
    return std::min(this->acceleration, this->motor_acceleration * (1.0F - speed / this->motor_speed));
}

float MotionLimits::crossover_speed() const {
    return this->motor_speed * (1.0F - this->acceleration / this->motor_acceleration);
}

Dynamics::Dynamics(const Config& config) :
    model{config.model},
    turns{config.turns},
    risky_turns{config.risky_turns},
    max_linear_speed{config.max_linear_speed},
    max_angular_speed{config.max_angular_speed},
    available_voltage{
        config.model.drive.supply_voltage * (1.0F - config.voltage_reserve) - config.model.drive.static_friction_voltage
    } { }

MotionLimits Dynamics::get_linear_limits(const RunProfile& profile) const {
    const float traction = profile.utilization * this->model.traction_acceleration(profile.fan);
    const float free_speed = this->available_voltage / this->model.speed_constant();

    return {
        .max_speed = std::min({profile.max_speed, this->max_linear_speed, 0.95F * free_speed}),
        .acceleration = traction,
        .deceleration = traction,
        .motor_acceleration = this->available_voltage / this->model.acceleration_constant(),
        .motor_speed = free_speed,
    };
}

MotionLimits Dynamics::get_angular_limits(const RunProfile& profile) const {
    const float traction = profile.utilization * this->model.traction_angular_acceleration(profile.fan);
    const float free_speed = this->available_voltage / this->model.angular_speed_constant();

    return {
        .max_speed = std::min(this->max_angular_speed, 0.95F * free_speed),
        .acceleration = traction,
        .deceleration = traction,
        .motor_acceleration = this->available_voltage / this->model.angular_acceleration_constant(),
        .motor_speed = free_speed,
    };
}

const TurnShape& Dynamics::get_turn(const RunProfile& profile, TurnId turn) const {
    return profile.risky ? this->risky_turns.get(turn) : this->turns.get(turn);
}

float Dynamics::get_turn_speed(const RunProfile& profile, TurnId turn) const {
    const TurnShape& shape = this->get_turn(profile, turn);

    const float lateral = profile.utilization * this->model.traction_acceleration(profile.fan);
    const float angular = profile.utilization * this->model.traction_angular_acceleration(profile.fan);

    return std::min(
        {std::sqrt(lateral / shape.curvature), std::sqrt(angular / shape.sharpness),
         this->get_linear_limits(profile).max_speed}
    );
}

const RobotModel& Dynamics::get_model() const {
    return this->model;
}
}  // namespace micras::nav
