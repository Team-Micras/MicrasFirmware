/**
 * @file follow_wall.cpp
 *
 * @brief Nav FollowWall class implementation
 *
 * @date 10/2024
 */

#include "micras/nav/follow_wall.hpp"

namespace micras::nav {
FollowWall::FollowWall(const proxy::WallSensors<4>& wall_sensors, const Config& config) :
    wall_sensors{wall_sensors},
    pid{config.pid},
    filter{config.cutoff_frequency},
    base_left_reading{config.base_left_reading},
    base_right_reading{config.base_right_reading} { }

float FollowWall::action(core::FollowWallType follow_wall_type, float elapsed_time) {
    float error{};

    switch (follow_wall_type) {
        case core::FollowWallType::FRONT:
            error = this->get_right_value() - this->get_left_value();
            break;

        case core::FollowWallType::LEFT:
            error = 2.0F * this->get_left_value();
            break;

        case core::FollowWallType::RIGHT:
            error = -2.0F * this->get_right_value();
            break;

        case core::FollowWallType::PARALLEL:
            error = this->get_left_value() - this->get_right_value();
            break;
        default:
            return 0.0F;
    }

    if (this->last_diff == 0.0F) {
        this->last_diff = error;
    }

    float response =
        this->pid.update(error, elapsed_time, this->filter.update((error - this->last_diff) / elapsed_time), true);

    this->last_diff = error;
    return response;
}

float FollowWall::get_left_value() const {
    return this->wall_sensors.get_reading(1) - this->base_left_reading;
}

float FollowWall::get_right_value() const {
    return this->wall_sensors.get_reading(2) - this->base_right_reading;
}

void FollowWall::reset() {
    this->pid.reset();
}

void FollowWall::reset_base_readings() {
    this->base_left_reading = this->wall_sensors.get_reading(1);
    this->base_right_reading = this->wall_sensors.get_reading(2);
}
}  // namespace micras::nav
