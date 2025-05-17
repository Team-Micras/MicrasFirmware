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
     */
    Action(const Id& action_id, bool follow_wall) : id{action_id}, follow_wall{follow_wall} { }

    /**
     * @brief Virtual destructor for the Action class.
     */
    virtual ~Action() = default;

    /**
     * @brief Get the desired speeds for the robot.
     *
     * @param pose The current pose of the robot.
     * @return The desired speeds for the robot.
     */
    virtual Twist get_speeds(const Pose& pose) const = 0;

    /**
     * @brief Check if the action is finished.
     *
     * @param pose The current pose of the robot.
     * @return True if the action is finished, false otherwise.
     */
    virtual bool finished(const Pose& pose) const = 0;

    /**
     * @brief Get the total time it takes to perform the action.
     *
     * @return The total time of the action in seconds.
     */
    virtual float get_total_time() const = 0;

    /**
     * @brief Increment the action ID value.
     *
     * @param value_increment The increment value.
     * @return A reference to the current action.
     */
    virtual Action& operator+=(float value_increment) {
        this->id.value += value_increment;
        return *this;
    }

    /**
     * @brief Decrement the action ID value.
     *
     * @param value_decrement The decrement value.
     * @return A reference to the current action.
     */
    virtual Action& operator-=(float value_decrement) {
        this->id.value -= value_decrement;
        return *this;
    }

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
     * @brief Time step for the action in seconds.
     */
    static constexpr float time_step{0.001F};

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
};
}  // namespace micras::nav

#endif  // MICRAS_NAV_BASE_ACTION_HPP
