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
    virtual Twist get_twist(const Pose& pose) const = 0;

    /**
     * @brief Check if the action is finished.
     *
     * @param pose The current pose of the robot.
     * @return True if the action is finished, false otherwise.
     */
    virtual bool finished(const Pose& pose) const = 0;

    /**
     * @brief Check if the action allow the robot to follow walls.
     *
     * @return True if the action allows the robot to follow walls, false otherwise.
     */
    virtual bool allow_follow_wall() const = 0;

protected:
    /**
     * @brief Special member functions declared as default.
     */
    ///@{
    Action() = default;
    Action(const Action&) = default;
    Action(Action&&) = default;
    Action& operator=(const Action&) = default;
    Action& operator=(Action&&) = default;
    ///@}
};
}  // namespace micras::nav

#endif  // MICRAS_NAV_BASE_ACTION_HPP
