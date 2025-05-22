/**
 * @file
 */

#ifndef MICRAS_NAV_BASE_ACTION_HPP
#define MICRAS_NAV_BASE_ACTION_HPP

#include "micras/nav/state.hpp"

namespace micras::nav {
/**
 * @brief Base class for actions.
 */
class Action {
public:
    struct Id {
        uint8_t type;
        float   value;
    };

    /**
     * @brief Constructor for the Action class.
     *
     * @param action_id The ID of the action.
     * @param follow_wall Whether the robot can follow walls while executing this action.
     * @param total_time The total time it takes to perform the action.
     */
    Action(const Id& action_id, bool follow_wall, float total_time = 0.0F) :
        id{action_id}, follow_wall{follow_wall}, total_time{total_time} { }

    /**
     * @brief Virtual destructor for the Action class.
     */
    virtual ~Action() = default;

    /**
     * @brief Get the desired speeds for the robot.
     *
     * @param current_pose The current pose of the robot.
     * @param time_step The time step for the action in seconds.
     * @return The desired speeds for the robot.
     */
    virtual Twist get_speeds(const Pose& current_pose, float time_step) = 0;

    /**
     * @brief Check if the action is finished.
     *
     * @param current_pose The current pose of the robot.
     * @return True if the action is finished, false otherwise.
     */
    virtual bool finished(const Pose& current_pose) = 0;

    /**
     * @brief Get the ID of the action.
     *
     * @return The ID of the action.
     */
    const Id& get_id() const { return id; }

    /**
     * @brief Check if the action allow the robot to follow walls.
     *
     * @return True if the action allows the robot to follow walls, false otherwise.
     */
    bool allow_follow_wall() const { return follow_wall; }

    /**
     * @brief Get the total time it takes to perform the action.
     *
     * @return The total time of the action in seconds.
     */
    float get_total_time() const { return this->total_time; };

    /**
     * @brief Set the total time it takes to perform the action.
     *
     * @param total_time The total time of the action in seconds.
     */
    void set_total_time(float total_time) { this->total_time = total_time; }

protected:
    /**
     * @brief Special member functions declared as default.
     */
    ///@{
    Action(const Action&) = default;
    Action(Action&&) = default;
    Action& operator=(const Action&) = default;
    Action& operator=(Action&&) = default;
    ///@}

private:
    /**
     * @brief The ID of the action.
     */
    Id id;

    /**
     * @brief Whether the robot can follow walls while executing this action.
     */
    bool follow_wall;

    /**
     * @brief The total time it takes to perform the action.
     */
    float total_time;
};
}  // namespace micras::nav

#endif  // MICRAS_NAV_BASE_ACTION_HPP
