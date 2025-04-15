/**
 * @file
 */

#include "micras/nav/follow_wall.hpp"

namespace micras::nav {
FollowWall::FollowWall(const std::shared_ptr<proxy::TWallSensors<4>>& wall_sensors, const Config& config) :
    wall_sensors{wall_sensors}, pid{config.pid}, max_linear_speed{config.max_linear_speed} { }

float FollowWall::action(core::Observation observation, float elapsed_time, float linear_speed) {
    float error{};

    if (observation.front) {
        error = this->wall_sensors->get_sensor_error(0) - this->wall_sensors->get_sensor_error(3);
    } else if (observation.left and observation.right) {
        error = this->wall_sensors->get_sensor_error(1) - this->wall_sensors->get_sensor_error(2);
    } else if (observation.left) {
        error = 2.0F * this->wall_sensors->get_sensor_error(1);
    } else if (observation.right) {
        error = 2.0F * this->wall_sensors->get_sensor_error(2);
    } else {
        error = 0.0F;
    }

    const float response = this->pid.update(error, elapsed_time);

    return response * linear_speed / this->max_linear_speed;
}

void FollowWall::reset() {
    this->pid.reset();
}
}  // namespace micras::nav
