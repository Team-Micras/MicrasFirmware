/**
 * @file
 */

#ifndef CALIBRATE_OFFSETS_STATE_HPP
#define CALIBRATE_OFFSETS_STATE_HPP

#include <cstdint>

#include "micras/states/base.hpp"

namespace micras {
/**
 * @brief State that measures what each wall sensor reads with nothing in front of it, and saves it.
 *
 * @note The robot is held up in open air, where no wall or floor is within range of the sensors.
 */
class CalibrateOffsetsState : public BaseState {
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

#endif  // CALIBRATE_OFFSETS_STATE_HPP
