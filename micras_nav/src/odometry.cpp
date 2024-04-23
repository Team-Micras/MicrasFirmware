/**
 * @file odometry.cpp
 *
 * @brief Nav Odometry class implementation
 *
 * @date 04/2024
 */

#include <cmath>
#include <numbers>

#include "micras/nav/odometry.hpp"

namespace micras::nav {
Odometry::Odometry(
    const proxy::RotarySensor& left_rotary_sensor, const proxy::RotarySensor& right_rotary_sensor,
    hal::Timer::Config timer_config, float wheel_radius, float wheel_separation
) :

    left_rotary_sensor{left_rotary_sensor},
    right_rotary_sensor{right_rotary_sensor},
    timer{timer_config},
    wheel_radius{wheel_radius},
    wheel_separation{wheel_separation} {
    this->left_last_position = this->left_rotary_sensor.get_position();
    this->right_last_position = this->right_rotary_sensor.get_position();
}

void Odometry::update() {
    float left_position = this->left_rotary_sensor.get_position();
    float right_position = this->right_rotary_sensor.get_position();

    float elapsed_time = this->timer.elapsed_time_us() / 1000000.0F;
    this->timer.reset_us();

    if (elapsed_time == 0) {
        return;
    }

    float left_distance = this->wheel_radius * (left_position - this->left_last_position);
    float right_distance = this->wheel_radius * (right_position - this->right_last_position);

    this->left_last_position = left_position;
    this->right_last_position = right_position;

    float linear_distance = (left_distance + right_distance) / 2;
    float angular_distance = (right_distance - left_distance) / this->wheel_separation;
    float linear_diagonal{};

    if (angular_distance < 0.05) {
        linear_diagonal = linear_distance;
    } else {
        linear_diagonal = std::abs(
            std::sqrt(1 - std::cos(angular_distance)) * std::numbers::sqrt2_v<float> * linear_distance /
            angular_distance
        );
    }

    this->position_x += linear_diagonal * std::cos(this->orientation + angular_distance / 2);
    this->position_y += linear_diagonal * std::sin(this->orientation + angular_distance / 2);

    this->orientation += angular_distance;

    this->linear_velocity = linear_distance / elapsed_time;
    this->angular_velocity = angular_distance / elapsed_time;
}

float Odometry::get_position_x() const {
    return this->position_x;
}

float Odometry::get_position_y() const {
    return this->position_y;
}

float Odometry::get_orientation() const {
    return this->orientation;
}

float Odometry::get_linear_velocity() const {
    return this->linear_velocity;
}

float Odometry::get_angular_velocity() const {
    return this->angular_velocity;
}
}  // namespace micras::nav
