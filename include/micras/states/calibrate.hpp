/**
 * @file
 */

#ifndef CALIBRATE_STATE_HPP
#define CALIBRATE_STATE_HPP

#include <utility>

#include "micras/states/base.hpp"

namespace micras {
class CalibrateState : public BaseState {
public:
    using BaseState::BaseState;

    /**
     * @brief Execute the entry function of this state.
     */
    void on_entry() override { this->micras.init(); }

    /**
     * @brief Execute this state.
     *
     * @return The id of the next state.
     */
    uint8_t execute() override {
        if (this->micras.calibrate()) {
            return std::to_underlying(Micras::State::IDLE);
        }

        return std::to_underlying(Micras::State::WAIT_FOR_CALIBRATE);
    }
};
}  // namespace micras

#endif  // CALIBRATE_STATE_HPP
