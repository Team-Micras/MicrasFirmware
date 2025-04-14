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
class BaseAction {
public:
    /**
     * @brief Virtual destructor for the BaseAction class.
     */
    virtual ~BaseAction() = default;

    virtual Twist get_twist(const State& state) const = 0;

    virtual bool finished(const State& state) const = 0;

protected:
    /**
     * @brief Special member functions declared as default.
     */
    ///@{
    BaseAction() = default;
    BaseAction(const BaseAction&) = default;
    BaseAction(BaseAction&&) = default;
    BaseAction& operator=(const BaseAction&) = default;
    BaseAction& operator=(BaseAction&&) = default;
    ///@}
};
}  // namespace micras::nav

#endif  // MICRAS_NAV_BASE_ACTION_HPP
