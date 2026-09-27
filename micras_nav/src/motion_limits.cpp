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

float MotionLimits::deceleration_at(float speed) const {
    return std::min(this->deceleration, this->motor_acceleration * (1.0F + speed / this->motor_speed));
}

float MotionLimits::braking_crossover_speed() const {
    return this->motor_speed * (this->deceleration / this->motor_acceleration - 1.0F);
}

MotionLimits MotionLimits::capped(float speed) const {
    MotionLimits limits = *this;
    limits.max_speed = std::min(limits.max_speed, speed);
    return limits;
}

float CurveLimits::get_speed_limit(const Bending& bending) const {
    const float curvature = std::abs(bending.curvature);
    const float sharpness = std::abs(bending.sharpness);

    float limit = this->linear.max_speed;

    if (curvature > 0.0F) {
        limit = std::min(limit, std::sqrt(this->lateral / curvature));
    }

    if (sharpness > 0.0F) {
        limit = std::min(limit, std::sqrt(this->angular / sharpness));
    }

    return limit;
}

float CurveLimits::get_acceleration(float speed, const Bending& bending) const {
    const float spare = this->get_spare(speed, bending);

    if (spare <= 0.0F) {
        return 0.0F;
    }

    const float grip = spare / (1.0F / this->linear.acceleration + std::abs(bending.curvature) / this->angular);

    return std::max(std::min(grip, this->linear.acceleration_at(speed)), 0.0F);
}

float CurveLimits::get_deceleration(float speed, const Bending& bending) const {
    const float spare = this->get_spare(speed, bending);

    if (spare <= 0.0F) {
        return 0.0F;
    }

    const float grip = spare / (1.0F / this->linear.deceleration + std::abs(bending.curvature) / this->angular);

    return std::min(grip, this->linear.deceleration_at(speed));
}

float CurveLimits::get_spare(float speed, const Bending& bending) const {
    const float lateral_use = speed * speed * std::abs(bending.curvature) / this->lateral;
    const float angular_use = speed * speed * std::abs(bending.sharpness) / this->angular;

    return std::sqrt(std::max(1.0F - lateral_use * lateral_use, 0.0F)) - angular_use;
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

CurveLimits Dynamics::get_curve_limits(const RunProfile& profile) const {
    return {
        .linear = this->get_linear_limits(profile),
        .lateral = profile.utilization * this->model.traction_acceleration(profile.fan),
        .angular = profile.utilization * this->model.traction_angular_acceleration(profile.fan),
    };
}

const TurnShape& Dynamics::get_turn(const RunProfile& profile, TurnId turn) const {
    return profile.risky ? this->risky_turns.get(turn) : this->turns.get(turn);
}

float Dynamics::get_turn_speed(const RunProfile& profile, TurnId turn) const {
    const TurnShape& shape = this->get_turn(profile, turn);

    return this->get_curve_limits(profile).get_speed_limit(
        {.curvature = shape.curvature, .sharpness = shape.sharpness}
    );
}

const RobotModel& Dynamics::get_model() const {
    return this->model;
}
}  // namespace micras::nav
