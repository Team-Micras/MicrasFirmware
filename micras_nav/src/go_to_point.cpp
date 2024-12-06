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
    stop_pid(config.stop_pid),
    angular_pid(config.angular_pid),
    follow_wall{wall_sensors, follow_wall_config},
    cell_size{config.cell_size},
    min_linear_command{config.min_linear_command},
    max_linear_command{config.max_linear_command},
    deceleration_factor{config.deceleration_factor},
    distance_tolerance{config.distance_tolerance},
    velocity_tolerance{config.velocity_tolerance} { }

Twist GoToPoint::action(
    const State& state, const Point& goal, core::FollowWallType follow_wall_type, float elapsed_time, bool stop
) {
    float linear_command{};
    float angular_command{};
    Point goal_distance = goal - state.pose.position;

    if (follow_wall_type != core::FollowWallType::NONE) {
        angular_command = this->follow_wall.action(follow_wall_type, elapsed_time);
    } else {
        float angular_error = core::assert_angle(state.pose.orientation - std::atan2(this->cell_size, goal_distance.x));
        angular_command = this->angular_pid.update(angular_error, elapsed_time, state.velocity.angular);
    }

    if (stop) {
        if (std::abs(goal_distance.y) < this->cell_size) {
            linear_command = this->stop_pid.update(-goal_distance.y, elapsed_time, state.velocity.linear);
        } else {
            linear_command =
                core::transition(goal_distance.y, 0.0F, this->max_linear_command, this->deceleration_factor);
        }
    } else {
        linear_command = core::transition(
            goal_distance.y, this->min_linear_command, this->max_linear_command, this->deceleration_factor
        );
    }

    angular_command *= linear_command / 50.0F;
    return {linear_command, angular_command};
}

bool GoToPoint::finished(const State& state, const Point& goal, bool stop) const {
    return std::abs((goal - state.pose.position).y) <= this->distance_tolerance and
           (not stop or std::abs(state.velocity.linear) <= this->velocity_tolerance);
}

void GoToPoint::reset() {
    this->follow_wall.reset();
    this->stop_pid.reset();
    this->angular_pid.reset();
}

void GoToPoint::calibrate() {
    this->follow_wall.reset_base_readings();
}
}  // namespace micras::nav
