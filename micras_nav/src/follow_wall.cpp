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
    sensor_index{config.wall_sensor_index},
    max_linear_speed{config.max_linear_speed},
    post_threshold{config.post_threshold},
    blind_pose{absolute_pose},
    cell_size{config.cell_size},
    post_clearance{config.post_clearance} { }

float FollowWall::compute_angular_correction(float elapsed_time, float linear_speed) {
    if (this->wall_sensors.use_count() == 1) {
        this->wall_sensors->update();
    }

    if (this->check_posts()) {
        return 0.0F;
    }

    if ((not this->following_left or not this->following_right) and
        (this->last_blind_distance >= (this->cell_size + (this->reset_by_post ? this->post_clearance : 0.0F)))) {
        this->reset();
    }

    float error{};

    if (this->following_left and this->following_right) {
        error = this->wall_sensors->get_sensor_error(this->sensor_index.left) -
                this->wall_sensors->get_sensor_error(this->sensor_index.right);
    } else if (this->following_left) {
        error = 2.0F * this->wall_sensors->get_sensor_error(this->sensor_index.left);
    } else if (this->following_right) {
        error = -2.0F * this->wall_sensors->get_sensor_error(this->sensor_index.right);
    } else {
        return 0.0F;
    }

    const float response = this->pid.compute_response(error, elapsed_time);

    return response * linear_speed / this->max_linear_speed;
}

bool FollowWall::check_posts() {
    const float current_distance = this->blind_pose.get().position.magnitude();
    const float delta_distance = current_distance - this->last_blind_distance;

    if (delta_distance <= 0.0F) {
        return false;
    }

    this->last_blind_distance = current_distance;
    bool found_posts = false;

    if (this->following_left and
        -(this->wall_sensors->get_sensor_error(this->sensor_index.left) - this->last_left_error) / delta_distance >=
            this->post_threshold) {
        this->following_left = false;

        if (this->following_right) {
            this->reset_displacement(true);
        }

        found_posts = true;
    }

    if (this->following_right and
        -(this->wall_sensors->get_sensor_error(this->sensor_index.right) - this->last_right_error) / delta_distance >=
            this->post_threshold) {
        this->following_right = false;

        if (this->following_left) {
            this->reset_displacement(true);
        }

        found_posts = true;
    }

    this->last_left_error = this->wall_sensors->get_sensor_error(this->sensor_index.left);
    this->last_right_error = this->wall_sensors->get_sensor_error(this->sensor_index.right);

    return found_posts;
}

core::Observation FollowWall::get_observation() const {
    const bool front_wall = this->wall_sensors->get_wall(this->sensor_index.left_front) and
                            this->wall_sensors->get_wall(this->sensor_index.right_front);
    const bool disturbed = front_wall;

    return {
        .left = this->wall_sensors->get_wall(this->sensor_index.left, disturbed),
        .front = front_wall,
        .right = this->wall_sensors->get_wall(this->sensor_index.right, disturbed),
    };
}

void FollowWall::reset() {
    this->pid.reset();
    this->reset_displacement();
    this->following_left = this->wall_sensors->get_wall(this->sensor_index.left);
    this->following_right = this->wall_sensors->get_wall(this->sensor_index.right);
}

void FollowWall::reset_displacement(bool reset_by_post) {
    this->blind_pose.reset_reference();
    this->last_blind_distance = 0.0F;
    this->reset_by_post = reset_by_post;
}
}  // namespace micras::nav
