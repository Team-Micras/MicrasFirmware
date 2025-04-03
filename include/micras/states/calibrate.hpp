/**
 * @file
 */

#ifndef CALIBRATE_STATE_HPP
#define CALIBRATE_STATE_HPP

#include "micras/core/fsm.hpp"
#include "micras/micras.hpp"

namespace micras {
class CalibrateState : public core::FSM::State {
public:
    CalibrateState(uint8_t id, Micras& micras);

    uint8_t run() override;

private:
    Micras& micras;  // NOLINT(cppcoreguidelines-avoid-const-or-ref-data-members)

    /**
     * @brief Calibrate the robot.
     */
    void calibrate();
};
}  // namespace micras

#endif  // CALIBRATE_STATE_HPP
