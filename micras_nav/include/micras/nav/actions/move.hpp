/**
 * @file
 */

#ifndef MICRAS_NAV_MOVE_ACTION_HPP
#define MICRAS_NAV_MOVE_ACTION_HPP

#include <cmath>

#include "micras/nav/actions/base.hpp"

namespace micras::nav {
/**
 * @brief Class to follow the side walls using a PID controller.
 */
class MoveAction : public Action {
public:
    MoveAction(
        float distance, float start_speed, float end_speed, float max_speed, float max_acceleration,
        float max_deceleration, bool follow_wall = true
    ) :
        distance(distance),
        start_speed_2(start_speed * start_speed),
        end_speed_2(end_speed * end_speed),
        max_speed{max_speed},
        max_acceleration_doubled{2.0F * max_acceleration},
        max_deceleration_doubled{2.0F * max_deceleration},
        follow_wall(follow_wall),
        decelerate_distance{
            (end_speed_2 - start_speed_2 + max_deceleration_doubled * distance) /
            (max_acceleration_doubled + max_deceleration_doubled)
        } { }

    Twist get_twist(const Pose& pose) const override {
        const float current_distance = pose.position.distance({0.0F, 0.0F});
        Twist       twist{};

        if (current_distance < this->decelerate_distance) {
            twist = {std::sqrt(this->start_speed_2 + this->max_acceleration_doubled * current_distance), 0.0F};
        } else {
            twist = {
                std::sqrt(this->end_speed_2 + this->max_deceleration_doubled * (this->distance - current_distance)),
                0.0F
            };
        }

        twist.linear = std::fminf(twist.linear, this->max_speed);

        return twist;
    }

    bool finished(const Pose& pose) const override { return pose.position.distance({0.0F, 0.0F}) >= this->distance; }

    bool allow_follow_wall() const override { return this->follow_wall; }

private:
    float distance;
    float start_speed_2;
    float end_speed_2;
    float max_speed;
    float max_acceleration_doubled;
    float max_deceleration_doubled;
    bool  follow_wall;

    float decelerate_distance;
};
}  // namespace micras::nav

#endif  // MICRAS_NAV_MOVE_ACTION_HPP
