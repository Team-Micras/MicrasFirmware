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
        Action{{action_type, distance}, follow_wall},
        distance(distance),
        start_speed(start_speed),
        end_speed(end_speed),
        max_speed{max_speed},
        max_acceleration{max_acceleration},
        max_deceleration{max_deceleration},
        total_time{calculate_total_time(distance, start_speed, end_speed, max_speed, max_acceleration, max_deceleration)
        },
        half_time{(end_speed - start_speed + max_deceleration * total_time) / (max_acceleration + max_deceleration)} { }

    /**
     * @brief Get the desired speeds for the robot to complete the action.
     *
     * @param time_step The time step for the action in seconds.
     * @return The desired speeds for the robot to complete the action.
     */
    Twist get_speeds(float time_step) override {
        this->elapsed_time += time_step;
        Twist twist{};

        if (this->elapsed_time < this->half_time) {
            twist = {
                .linear = this->start_speed + this->max_acceleration * this->elapsed_time,
                .angular = 0.0F,
            };
        } else {
            twist = {
                .linear = this->end_speed + this->max_deceleration * (this->total_time - this->elapsed_time),
                .angular = 0.0F,
            };
        }

        twist.linear = std::clamp(twist.linear, 0.0F, this->max_speed);

        return twist;
    }

    /**
     * @brief Check if the action is finished.
     *
     * @return True if the action is finished, false otherwise.
     */
    bool finished() const override { return this->elapsed_time >= this->total_time; }

    /**
     * @brief Get the total time it takes to perform the action.
     *
     * @return The total time of the action in seconds.
     */
    float get_total_time() const override { return this->total_time; }

    /**
     * @brief Increment the distance to move by a certain value.
     *
     * @param value_increment The increment value.
     * @return A reference to the current action.
     */
    MoveAction& operator+=(float value_increment) override {
        Action::operator+=(value_increment);
        this->distance += value_increment;

        this->total_time = calculate_total_time(
            this->distance, this->start_speed, this->end_speed, this->max_speed, this->max_acceleration,
            this->max_deceleration
        );

        this->half_time = (this->end_speed - this->start_speed + this->max_deceleration * this->total_time) /
                          (this->max_acceleration + this->max_deceleration);

        return *this;
    }

    /**
     * @brief Decrement the distance to move by a certain value.
     *
     * @param value_increment The decrement value.
     * @return A reference to the current action.
     */
    MoveAction& operator-=(float value_increment) override { return *this += -value_increment; }

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
     * @brief Initial linear speed in m/s.
     */
    float start_speed;

    /**
     * @brief Final linear speed in m/s.
     */
    float end_speed;

    /**
     * @brief Maximum linear speed in m/s.
     */
    float max_speed;

    /**
     * @brief Maximum linear acceleration.
     */
    float max_acceleration;

    /**
     * @brief Maximum linear deceleration.
     */
    float max_deceleration;

    /**
     * @brief Total time to complete the action in seconds.
     */
    float total_time;

    /**
     * @brief Threshold time to start the deceleration phase.
     */
    float half_time{};

    /**
     * @brief Elapsed time in seconds since the action started.
     */
    float elapsed_time{};
};
}  // namespace micras::nav

#endif  // MICRAS_NAV_MOVE_ACTION_HPP
