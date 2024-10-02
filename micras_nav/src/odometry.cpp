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
    const proxy::Imu& imu, hal::Timer::Config timer_config, Config config
) :
    left_rotary_sensor{left_rotary_sensor},
    right_rotary_sensor{right_rotary_sensor},
    imu{imu},
    timer{timer_config},
    wheel_radius{config.wheel_radius},
    wheel_separation{config.wheel_separation},
    left_last_position{left_rotary_sensor.get_position()},
    right_last_position{right_rotary_sensor.get_position()},
    linear_filter{config.linear_cutoff_frequency},
    angular_filter{config.angular_cutoff_frequency},
    state{config.initial_pose, {0.0F, 0.0F}},
    imu_state{config.initial_pose, {0.0F, 0.0F}} { }

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
    float half_angle = angular_distance / 2;
    float linear_diagonal =
        angular_distance < 0.05F ? linear_distance : std::abs(std::sin(half_angle) * linear_distance / half_angle);

    this->state.pose.position.x += linear_diagonal * std::cos(this->state.pose.orientation + half_angle);
    this->state.pose.position.y += linear_diagonal * std::sin(this->state.pose.orientation + half_angle);

    this->state.pose.orientation += angular_distance;

    this->state.velocity.linear = this->linear_filter.update(linear_distance / elapsed_time);
    this->state.velocity.angular = this->angular_filter.update(angular_distance / elapsed_time);

    angular_distance = this->imu.get_angular_velocity(proxy::Imu::Axis::Z) * elapsed_time;
    half_angle = angular_distance / 2;
    linear_diagonal =
        angular_distance < 0.05F ? linear_distance : std::abs(std::sin(half_angle) * linear_distance / half_angle);

    this->imu_state.pose.position.x += linear_diagonal * std::cos(this->imu_state.pose.orientation + half_angle);
    this->imu_state.pose.position.y += linear_diagonal * std::sin(this->imu_state.pose.orientation + half_angle);

    this->imu_state.pose.orientation += angular_distance;

    this->imu_state.velocity.linear = linear_distance / elapsed_time;
    this->imu_state.velocity.angular = this->imu.get_angular_velocity(proxy::Imu::Axis::Z);
}

const nav::State& Odometry::get_state() const {
    return this->state;
}

const nav::State& Odometry::get_imu_state() const {
    return this->imu_state;
}
}  // namespace micras::nav
