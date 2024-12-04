/**
 * @file
 */

#include <cmath>

#include "micras/core/utils.hpp"
#include "micras/nav/go_to_point.hpp"

namespace micras::nav {
GoToPoint::GoToPoint(
    const proxy::WallSensors<4>& wall_sensors, const Config& config, const FollowWall::Config& follow_wall_config
) :
    linear_pid(config.linear_pid),
    stop_pid(config.stop_pid),
    angular_pid(config.angular_pid),
    follow_wall{wall_sensors, follow_wall_config},
    cell_size{config.cell_size},
    base_speed{config.base_speed},
    linear_decay_damping{config.linear_decay_damping},
    distance_tolerance{config.distance_tolerance},
    velocity_tolerance{config.velocity_tolerance} { }

Twist GoToPoint::action(
    const State& state, const Point& goal, core::FollowWallType follow_wall_type, float elapsed_time, bool stop
) {
    float linear_command{};
    float angular_command{};
    Point goal_distance = goal - state.pose.position;

    float linear_target{};

    if (follow_wall_type != core::FollowWallType::NONE) {
        linear_target = this->base_speed;
        angular_command = this->follow_wall.action(follow_wall_type, elapsed_time);
    } else {
        float angular_error = core::assert_angle(state.pose.orientation - std::atan2(this->cell_size, goal_distance.x));
        linear_target = core::decay(angular_error, this->linear_decay_damping) * this->base_speed;
        angular_command = this->angular_pid.update(angular_error, elapsed_time, state.velocity.angular);
    }

    if (stop and (std::abs(goal_distance.y) < this->cell_size / 2.0F)) {
        linear_command = this->stop_pid.update(-goal_distance.y, elapsed_time, state.velocity.linear);
    } else {
        linear_command = this->linear_pid.update(state.velocity.linear - linear_target, elapsed_time);
    }

    return {linear_command, angular_command};
}

bool GoToPoint::finished(const State& state, const Point& goal, bool stop) const {
    return std::abs((goal - state.pose.position).y) <= this->distance_tolerance and
           (not stop or std::abs(state.velocity.linear) <= this->velocity_tolerance);
}

void GoToPoint::reset() {
    this->follow_wall.reset();
    this->linear_pid.reset();
    this->stop_pid.reset();
    this->angular_pid.reset();
}

void GoToPoint::calibrate() {
    this->follow_wall.reset_base_readings();
}
}  // namespace micras::nav
