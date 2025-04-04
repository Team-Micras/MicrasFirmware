/**
 * @file
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
    left_last_position{left_rotary_sensor.get_position()},
    right_last_position{right_rotary_sensor.get_position()},
    linear_filter{config.linear_cutoff_frequency},
    state{config.initial_pose, {0.0F, 0.0F}} { }

void Odometry::update(float elapsed_time) {
    const float left_position = this->left_rotary_sensor.get_position();
    const float right_position = this->right_rotary_sensor.get_position();

    const float left_distance = this->wheel_radius * (left_position - this->left_last_position);
    const float right_distance = this->wheel_radius * (right_position - this->right_last_position);

    this->left_last_position = left_position;
    this->right_last_position = right_position;

    const float linear_distance = (left_distance + right_distance) / 2;

    this->state.velocity.linear = this->linear_filter.update(linear_distance / elapsed_time);
    this->state.velocity.angular = this->imu.get_angular_velocity(proxy::Imu::Axis::Z);

    const float angular_distance = this->state.velocity.angular * elapsed_time;

    const float half_angle = angular_distance / 2;
    const float linear_diagonal =
        angular_distance < 0.05F ? linear_distance : std::abs(std::sin(half_angle) * linear_distance / half_angle);

    this->state.pose.position.x += linear_diagonal * std::cos(this->state.pose.orientation + half_angle);
    this->state.pose.position.y += linear_diagonal * std::sin(this->state.pose.orientation + half_angle);

    this->state.pose.orientation += angular_distance;
}

void Odometry::reset() {
    this->left_last_position = this->left_rotary_sensor.get_position();
    this->right_last_position = this->right_rotary_sensor.get_position();
}

const nav::State& Odometry::get_state() const {
    return this->state;
}

void Odometry::set_state(const nav::State& new_state) {
    this->state = new_state;
}
}  // namespace micras::nav
