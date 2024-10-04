/**
 * @file odometry.cpp
 *
 * @brief Nav Odometry class implementation
 *
 * @date 04/2024
 */

#include <cmath>
#include <numbers>

#include "micras/core/utils.hpp"
#include "micras/nav/odometry.hpp"

namespace micras::nav {
Odometry::Odometry(
    const proxy::RotarySensor& left_rotary_sensor, const proxy::RotarySensor& right_rotary_sensor,
    const proxy::Imu& imu, Config config
) :
    left_rotary_sensor{left_rotary_sensor},
    right_rotary_sensor{right_rotary_sensor},
    imu{imu},
    wheel_radius{config.wheel_radius},
    wheel_separation{config.wheel_separation},
    left_last_position{left_rotary_sensor.get_position()},
    right_last_position{right_rotary_sensor.get_position()},
    linear_filter{config.linear_cutoff_frequency},
    angular_filter{config.angular_cutoff_frequency},
    state{config.initial_pose, {0.0F, 0.0F}},
    imu_state{config.initial_pose, {0.0F, 0.0F}} { }

void Odometry::update(float elapsed_time) {
    float left_position = this->left_rotary_sensor.get_position();
    float right_position = this->right_rotary_sensor.get_position();

    float left_distance = this->wheel_radius * (left_position - this->left_last_position);
    float right_distance = this->wheel_radius * (right_position - this->right_last_position);

    this->left_last_position = left_position;
    this->right_last_position = right_position;

    float linear_distance = (left_distance + right_distance) / 2;
    float angular_distance = (right_distance - left_distance) / this->wheel_separation;
    float linear_velocity = this->linear_filter.update(linear_distance / elapsed_time);
    float angular_velocity = this->angular_filter.update(angular_distance / elapsed_time);
    update_state(this->state, linear_distance, angular_distance, linear_velocity, angular_velocity);

    angular_velocity = this->imu.get_angular_velocity(proxy::Imu::Axis::Z);
    angular_distance = angular_velocity * elapsed_time;
    update_state(this->imu_state, linear_distance, angular_distance, linear_velocity, angular_velocity);
}

const nav::State& Odometry::get_state() const {
    return this->state;
}

const nav::State& Odometry::get_imu_state() const {
    return this->imu_state;
}
}  // namespace micras::nav
