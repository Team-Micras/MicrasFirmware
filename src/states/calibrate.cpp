/**
 * @file
 */

#include <cstdint>
#include <utility>

#include "micras/micras.hpp"
#include "micras/states/base.hpp"
#include "micras/states/calibrate.hpp"

namespace micras {
void CalibrateState::on_entry() {
    this->micras.start_calibration();
}

uint8_t CalibrateState::execute() {
    if (not this->micras.calibrate()) {
        return this->get_id();
    }

    return std::to_underlying(this->micras.is_calibration_complete() ? State::IDLE : State::WAIT_FOR_CALIBRATE);
}
}  // namespace micras
