/**
 * @file
 */

#ifndef INIT_STATE_HPP
#define INIT_STATE_HPP

#include <utility>

#include "micras/states/base.hpp"

namespace micras {
class InitState : public BaseState {
public:
    using BaseState::BaseState;

    /**
     * @brief Execute this state.
     *
     * @return The id of the next state.
     */
    uint8_t execute() override {
        if (not this->micras.check_initialization()) {
            return std::to_underlying(Micras::State::ERROR);
        }

        return std::to_underlying(Micras::State::IDLE);
    }
};
}  // namespace micras

#endif  // INIT_STATE_HPP
