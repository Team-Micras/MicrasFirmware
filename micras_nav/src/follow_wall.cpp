/**
 * @file
 */

#include "micras/nav/follow_wall.hpp"

namespace micras::nav {
FollowWall::FollowWall(
    const std::shared_ptr<proxy::TWallSensors<4>>& wall_sensors, const Pose& absolute_pose, const Config& config
) :
    wall_sensors{wall_sensors},
    pid{config.pid},
    max_linear_speed{config.max_linear_speed},
    post_threshold{config.post_threshold},
    blind_pose{absolute_pose},
    cell_size{config.cell_size},
    post_margin{config.post_margin} { }

float FollowWall::action(float elapsed_time, float linear_speed) {
    if (this->check_posts()) {
        return 0.0F;
    }

    if ((not this->left_wall or not this->right_wall) and
        (this->last_blind_distance >= (this->cell_size + (this->reset_by_post ? this->post_margin : 0.0F)))) {
        this->left_wall = this->wall_sensors->get_wall(0);
        this->right_wall = this->wall_sensors->get_wall(3);
        this->reset_displacement();
    }

    float error{};

    if (this->left_wall and this->right_wall) {
        error = this->wall_sensors->get_sensor_error(1) - this->wall_sensors->get_sensor_error(2);
    } else if (this->left_wall) {
        error = 2.0F * this->wall_sensors->get_sensor_error(1);
    } else if (this->right_wall) {
        error = -2.0F * this->wall_sensors->get_sensor_error(2);
    } else {
        return 0.0F;
    }

    const float response = this->pid.update(error, elapsed_time);

    return response * linear_speed / this->max_linear_speed;
}

bool FollowWall::check_posts() {
    const float current_distance = this->blind_pose.get().position.distance({0.0F, 0.0F});
    const float delta_distance = current_distance - this->last_blind_distance;

    if (delta_distance <= 0.0F) {
        return false;
    }

    this->last_blind_distance = current_distance;
    bool found_posts = false;

    if (this->left_wall and
        (this->wall_sensors->get_sensor_error(1) - this->last_left_error) / delta_distance <= this->post_threshold) {
        this->left_wall = false;

        if (this->right_wall) {
            this->reset_displacement(true);
        }

        found_posts = true;
    }

    if (this->right_wall and
        (this->wall_sensors->get_sensor_error(2) - this->last_right_error) / delta_distance <= this->post_threshold) {
        this->right_wall = false;

        if (this->left_wall) {
            this->reset_displacement(true);
        }

        found_posts = true;
    }

    this->last_left_error = this->wall_sensors->get_sensor_error(1);
    this->last_right_error = this->wall_sensors->get_sensor_error(2);

    return found_posts;
}

void FollowWall::reset_displacement(bool reset_by_post) {
    this->blind_pose.reset_reference();
    this->last_blind_distance = 0.0F;
    this->reset_by_post = reset_by_post;
}

void FollowWall::reset() {
    this->pid.reset();
    this->reset_displacement();
    this->left_wall = this->wall_sensors->get_wall(1);
    this->right_wall = this->wall_sensors->get_wall(2);
}
}  // namespace micras::nav
