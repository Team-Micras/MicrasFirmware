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
        float max_deceleration
    ) :
        distance(distance),
        start_speed_2(start_speed * start_speed),
        end_speed_2(end_speed * end_speed),
        max_speed{max_speed},
        max_acceleration_doubled{2.0F * max_acceleration},
        max_deceleration_doubled{2.0F * max_deceleration},
        decelerate_distance{
            (end_speed_2 - start_speed_2 + max_deceleration_doubled * distance) /
            (max_acceleration_doubled + max_deceleration_doubled)
        } { }

    Twist get_twist(const State& state) const override {
        const float current_distance = state.pose.position.distance({0.0F, 0.0F});
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

    bool finished(const State& state) const override {
        return state.pose.position.distance({0.0F, 0.0F}) >= this->distance;
    }

private:
    float distance;
    float start_speed_2;
    float end_speed_2;
    float max_speed;
    float max_acceleration_doubled;
    float max_deceleration_doubled;

    float decelerate_distance;
};
}  // namespace micras::nav

#endif  // MICRAS_NAV_MOVE_ACTION_HPP
