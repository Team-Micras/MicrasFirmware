/**
 * @file
 */

#ifndef CALIBRATE_GYROSCOPE_STATE_HPP
#define CALIBRATE_GYROSCOPE_STATE_HPP

#include <cstdint>

#include "micras/states/base.hpp"

namespace micras {
/**
 * @brief State that measures the scale factor of the gyroscope.
 */
class CalibrateGyroscopeState : public BaseState {
public:
    using BaseState::BaseState;

    /**
     * @brief Execute the entry function of this state.
     */
    void on_entry() override;

    /**
     * @brief Execute this state.
     *
     * @return The id of the next state.
     */
    uint8_t execute() override;
};
}  // namespace micras

#endif  // CALIBRATE_GYROSCOPE_STATE_HPP
