/**
 * @file
 */

#include <array>
#include <cmath>
#include <cstdint>
#include <utility>

#include "micras/nav/controller.hpp"
#include "micras/nav/drive_identification.hpp"
#include "micras/nav/measurements.hpp"
#include "micras/nav/robot_model.hpp"

namespace micras::nav {
DriveIdentification::DriveIdentification(const Config& config) : config{config} { }

void DriveIdentification::start(const Measurements& measurements) {
    this->phase = Phase::RAMP;
    this->phase_time = 0.0F;
    this->rest_left = this->config.rest_time;
    this->last_left_angle = measurements.left_wheel_angle;
    this->last_right_angle = measurements.right_wheel_angle;
    this->distance = 0.0F;
    this->breakaway_voltage = 0.0F;
    this->linear_fit = {};
    this->angular_fit = {};
    this->valid = false;
}

Controller::Command DriveIdentification::update(const Measurements& measurements, float elapsed_time) {
    const RobotModel& model = this->config.model;

    const float radius = model.chassis.wheel_radius;
    const float left = radius * (measurements.left_wheel_angle - this->last_left_angle);
    const float right = radius * (measurements.right_wheel_angle - this->last_right_angle);

    this->last_left_angle = measurements.left_wheel_angle;
    this->last_right_angle = measurements.right_wheel_angle;

    const float linear_speed = (left + right) / (2.0F * elapsed_time);
    const float angular_speed = model.gyroscope_scale * measurements.angular_rate;

    this->distance += (left + right) / 2.0F;

    if (this->phase == Phase::FINISHED) {
        return {};
    }

    if (this->rest_left > 0.0F) {
        this->rest_left -= elapsed_time;
        this->start_speed = this->phase == Phase::LEFT or this->phase == Phase::RIGHT ? angular_speed : linear_speed;
        return {};
    }

    this->phase_time += elapsed_time;

    const float to_volts = model.drive.supply_voltage / 100.0F;

    if (this->phase == Phase::RAMP) {
        const float command = this->config.ramp_rate * this->phase_time;

        if (std::abs(linear_speed) > this->config.breakaway_speed or command >= this->config.linear_command) {
            this->breakaway_voltage = command * to_volts;
            this->advance(measurements);
            return {};
        }

        return {.forward = command, .rotation = 0.0F};
    }

    const bool  rotating = this->phase == Phase::LEFT or this->phase == Phase::RIGHT;
    const bool  positive = this->phase == Phase::FORWARD or this->phase == Phase::LEFT;
    const float level = this->phase_time < this->config.step_time ? 1.0F : second_level;
    const float magnitude = level * (rotating ? this->config.angular_command : this->config.linear_command);
    const float command = positive ? magnitude : -magnitude;
    const float speed = rotating ? angular_speed : linear_speed;

    this->voltage_integral += command * to_volts * elapsed_time;
    this->sign_integral += (speed != 0.0F ? std::copysign(1.0F, speed) : 0.0F) * elapsed_time;
    this->position += speed * elapsed_time;

    (rotating ? this->angular_fit : this->linear_fit)
        .add({this->sign_integral, this->position, speed - this->start_speed}, this->voltage_integral);

    const bool too_far =
        not rotating and std::abs(this->distance) > this->config.max_distance and (this->distance > 0.0F) == positive;

    if (this->phase_time >= 2.0F * this->config.step_time or too_far) {
        this->advance(measurements);
        return {};
    }

    return rotating ? Controller::Command{.forward = 0.0F, .rotation = command} :
                      Controller::Command{.forward = command, .rotation = 0.0F};
}

bool DriveIdentification::is_finished() const {
    return this->phase == Phase::FINISHED;
}

bool DriveIdentification::is_valid() const {
    return this->valid;
}

float DriveIdentification::get_breakaway_voltage() const {
    return this->breakaway_voltage;
}

const DriveIdentification::Axis& DriveIdentification::get_linear() const {
    return this->linear;
}

const DriveIdentification::Axis& DriveIdentification::get_angular() const {
    return this->angular;
}

RobotModel DriveIdentification::get_model() const {
    RobotModel model = this->config.model;

    const RobotModel::Chassis& chassis = model.chassis;

    model.drive.static_friction_voltage = this->linear.static_friction;
    model.drive.torque_constant = this->linear.speed_constant * chassis.wheel_radius / model.drive.gear_ratio;
    model.drive.resistance = this->linear.acceleration_constant * 2.0F * model.drive.gear_ratio *
                             model.drive.torque_constant / (chassis.wheel_radius * chassis.mass);
    model.chassis.yaw_inertia = this->angular.acceleration_constant * model.drive.gear_ratio *
                                model.drive.torque_constant * chassis.track_width /
                                (chassis.wheel_radius * model.drive.resistance);

    return model;
}

void DriveIdentification::Fit::add(const std::array<float, 3>& regressors, float response) {
    for (uint8_t i = 0; i < 3; i++) {
        for (uint8_t j = 0; j < 3; j++) {
            this->normal.at(i).at(j) += static_cast<double>(regressors.at(i)) * static_cast<double>(regressors.at(j));
        }

        this->moment.at(i) += static_cast<double>(regressors.at(i)) * static_cast<double>(response);
    }
}

bool DriveIdentification::Fit::solve(Axis& axis) const {
    std::array<std::array<double, 4>, 3> system{};

    for (uint8_t i = 0; i < 3; i++) {
        for (uint8_t j = 0; j < 3; j++) {
            system.at(i).at(j) = this->normal.at(i).at(j);
        }

        system.at(i).at(3) = this->moment.at(i);
    }

    for (uint8_t column = 0; column < 3; column++) {
        uint8_t pivot = column;

        for (uint8_t row = column + 1; row < 3; row++) {
            if (std::abs(system.at(row).at(column)) > std::abs(system.at(pivot).at(column))) {
                pivot = row;
            }
        }

        if (std::abs(system.at(pivot).at(column)) < 1.0e-12) {
            return false;
        }

        std::swap(system.at(column), system.at(pivot));

        for (uint8_t row = 0; row < 3; row++) {
            if (row == column) {
                continue;
            }

            const double factor = system.at(row).at(column) / system.at(column).at(column);

            for (uint8_t k = column; k < 4; k++) {
                system.at(row).at(k) -= factor * system.at(column).at(k);
            }
        }
    }

    axis = {
        .static_friction = static_cast<float>(system.at(0).at(3) / system.at(0).at(0)),
        .speed_constant = static_cast<float>(system.at(1).at(3) / system.at(1).at(1)),
        .acceleration_constant = static_cast<float>(system.at(2).at(3) / system.at(2).at(2)),
    };

    return axis.speed_constant > 0.0F and axis.acceleration_constant > 0.0F;
}

void DriveIdentification::advance(const Measurements& /*measurements*/) {
    this->phase = static_cast<Phase>(static_cast<uint8_t>(this->phase) + 1);
    this->phase_time = 0.0F;
    this->rest_left = this->config.rest_time;
    this->voltage_integral = 0.0F;
    this->sign_integral = 0.0F;
    this->position = 0.0F;

    if (this->phase == Phase::FINISHED) {
        const bool linear_found = this->linear_fit.solve(this->linear);
        const bool angular_found = this->angular_fit.solve(this->angular);

        this->valid = linear_found and angular_found;
    }
}
}  // namespace micras::nav
