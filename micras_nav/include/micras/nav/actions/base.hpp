/**
 * @file
 */

#ifndef MICRAS_NAV_BASE_ACTION_HPP
#define MICRAS_NAV_BASE_ACTION_HPP

#include "micras/nav/state.hpp"

namespace micras::nav {
/**
 * @brief Class to follow the side walls using a PID controller.
 */
class Action {
public:
    /**
     * @brief Virtual destructor for the Action class.
     */
    virtual ~Action() = default;

    virtual Twist get_twist(const Pose& pose) const = 0;

    virtual bool finished(const Pose& pose) const = 0;

    virtual constexpr bool allow_follow_wall() const = 0;

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
