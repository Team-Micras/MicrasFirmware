/**
 * @file
 */

#ifndef MICRAS_NAV_MOVE_ACTION_HPP
#define MICRAS_NAV_MOVE_ACTION_HPP

#include <cmath>

#include "micras/nav/actions/base.hpp"

namespace micras::nav {
/**
 * @brief Action to move the robot a certain distance forward.
 */
class MoveAction : public Action {
public:
    /**
     * @brief Construct a new Move Action object.
     *
     * @param action_type The type of the action to be performed.
     * @param distance Distance to move in meters.
     * @param start_speed Initial speed in m/s.
     * @param end_speed Final speed in m/s.
     * @param max_speed Maximum speed in m/s.
     * @param max_acceleration Maximum acceleration in m/s^2.
     * @param max_deceleration Maximum deceleration in m/s^2.
     * @param follow_wall Whether the robot can follow wall while executing this action.
     */
    MoveAction(
        uint8_t action_type, float distance, float start_speed, float end_speed, float max_speed,
        float max_acceleration, float max_deceleration, bool follow_wall = true
    ) :
        Action{
            {action_type, distance},
            follow_wall,
            calculate_total_time(distance, start_speed, end_speed, max_speed, max_acceleration, max_deceleration)
        },
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

    /**
     * @brief Get the desired speeds for the robot to complete the action.
     *
     * @param current_pose The current pose of the robot.
     * @param time_step The time step for the action in seconds.
     * @return The desired speeds for the robot to complete the action.
     *
     * @details The desired velocity is calculated from the linear displacement based on the Torricelli equation.
     */
    Twist get_speeds(const Pose& current_pose, float time_step) override {
        const float current_distance = current_pose.position.magnitude();
        Twist       twist{};

        if (current_distance < this->decelerate_distance) {
            twist = {
                .linear = std::sqrt(this->start_speed_2 + this->max_acceleration_doubled * current_distance) +
                          time_step * this->max_acceleration_doubled / 2.0F,
                .angular = 0.0F,
            };
        } else {
            twist = {
                .linear =
                    std::sqrt(this->end_speed_2 + this->max_deceleration_doubled * (this->distance - current_distance)),
                .angular = 0.0F,
            };
        }

        twist.linear = std::clamp(twist.linear, 0.0F, this->max_speed);

        return twist;
    }

    /**
     * @brief Check if the action is finished.
     *
     * @param pose The current pose of the robot.
     * @return True if the action is finished, false otherwise.
     */
    bool finished(const Pose& pose) override { return pose.position.magnitude() >= this->distance; }

private:
    /**
     * @brief Calculate the total time to complete the action.
     *
     * @param distance Distance to move in meters.
     * @param start_speed Initial speed in m/s.
     * @param end_speed Final speed in m/s.
     * @param max_speed Maximum speed in m/s.
     * @param max_acceleration Maximum acceleration in m/s^2.
     * @param max_deceleration Maximum deceleration in m/s^2.
     * @return Total time to complete the action in seconds.
     */
    static constexpr float calculate_total_time(
        float distance, float start_speed, float end_speed, float max_speed, float max_acceleration,
        float max_deceleration
    ) {
        const float peak_velocity = std::sqrt(
            (2.0F * distance * max_acceleration * max_deceleration + start_speed * start_speed * max_deceleration +
             end_speed * end_speed * max_acceleration) /
            (max_acceleration + max_deceleration)
        );

        if (peak_velocity < max_speed) {
            return (peak_velocity - start_speed) / max_acceleration + (peak_velocity - end_speed) / max_deceleration;
        }

        const float acceleration_time = (max_speed - start_speed) / max_acceleration;
        const float deceleration_time = (max_speed - end_speed) / max_deceleration;

        const float acceleration_distance =
            (start_speed * start_speed + max_speed * max_speed) / (2.0F * max_acceleration);
        const float deceleration_distance = (max_speed * max_speed + end_speed * end_speed) / (2.0F * max_deceleration);
        const float constant_speed_distance = distance - acceleration_distance - deceleration_distance;
        const float constant_speed_time = constant_speed_distance / max_speed;

        return acceleration_time + deceleration_time + constant_speed_time;
    }

    /**
     * @brief Distance to move in meters.
     */
    float distance;

    /**
     * @brief Initial linear speed squared.
     */
    float start_speed_2;

    /**
     * @brief Final linear speed squared.
     */
    float end_speed_2;

    /**
     * @brief Maximum linear speed in m/s.
     */
    float max_speed;

    /**
     * @brief Maximum linear acceleration multiplied by 2.
     */
    float max_acceleration_doubled;

    /**
     * @brief Maximum linear deceleration multiplied by 2.
     */
    float max_deceleration_doubled;

    /**
     * @brief Distance from the start where the robot should start to decelerate in meters.
     */
    float decelerate_distance;
};
}  // namespace micras::nav

#endif  // MICRAS_NAV_MOVE_ACTION_HPP
