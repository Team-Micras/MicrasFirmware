/**
 * @file
 */

#include "micras/nav/follow_wall.hpp"

namespace micras::nav {
FollowWall::FollowWall(const std::shared_ptr<proxy::TWallSensors<4>>& wall_sensors, const Config& config) :
    wall_sensors{wall_sensors},
    pid{config.pid},
    sensor_index{config.wall_sensor_index},
    max_linear_speed{config.max_linear_speed},
    max_angular_acceleration{config.max_angular_acceleration},
    cell_size{config.cell_size},
    post_threshold{config.post_threshold},
    post_clearance{config.post_clearance},
    post_reference{config.post_reference} { }

float FollowWall::compute_angular_correction(float elapsed_time, State& state) {
    float          cell_advance = state.pose.to_cell(this->cell_size).y;
    const GridPose grid_pose = state.pose.to_grid(this->cell_size);

    if (this->wall_sensors.use_count() == 1) {
        this->wall_sensors->update();
    }

    if (grid_pose == this->last_grid_pose) {
        if (this->check_posts(cell_advance)) {
            clear_position_error(state, this->post_reference - cell_advance);
            cell_advance = this->post_reference;
        }
    } else {
        if (grid_pose != this->last_grid_pose.front()) {
            this->reset();
        }
    }

    this->last_grid_pose = grid_pose;
    this->last_cell_advance = cell_advance;

    if (std::abs(cell_advance - this->cell_size / 2.0F) < this->cell_size / 2.0F - this->post_clearance) {
        this->following_left |= this->wall_sensors->get_wall(this->sensor_index.left);
        this->following_right |= this->wall_sensors->get_wall(this->sensor_index.right);
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

    const float response =
        state.velocity.linear * this->pid.compute_response(error, elapsed_time) / this->max_linear_speed;
    const float clamped_response =
        core::move_towards(this->last_response, response, this->max_angular_acceleration * elapsed_time);

    this->last_response = clamped_response;
    return clamped_response;
}

bool FollowWall::check_posts(float cell_advance) {
    const float delta_distance = cell_advance - this->last_cell_advance;

    if (delta_distance <= 0.0F) {
        return false;
    }

    bool found_posts = false;

    if (-(this->wall_sensors->get_reading(this->sensor_index.left) - this->last_left_reading) / delta_distance >=
        this->post_threshold) {
        this->following_left = false;
        found_posts = true;
    }

    if (-(this->wall_sensors->get_reading(this->sensor_index.right) - this->last_right_reading) / delta_distance >=
        this->post_threshold) {
        this->following_right = false;
        found_posts = true;
    }

    this->last_left_reading = this->wall_sensors->get_reading(this->sensor_index.left);
    this->last_right_reading = this->wall_sensors->get_reading(this->sensor_index.right);

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

void FollowWall::clear_position_error(State& state, float error) {
    switch (angle_to_grid(state.pose.orientation)) {
        case Side::RIGHT:
            state.pose.position.x += error;
            break;
        case Side::UP:
            state.pose.position.y += error;
            break;
        case Side::LEFT:
            state.pose.position.x -= error;
            break;
        case Side::DOWN:
            state.pose.position.y -= error;
            break;
    }
}

void FollowWall::reset() {
    this->pid.reset();
    this->last_response = 0.0F;
    this->following_left = false;
    this->following_right = false;
    this->last_left_reading = 0.0F;
    this->last_right_reading = 0.0F;
}
}  // namespace micras::nav
