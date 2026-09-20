/**
 * @file
 */

#include <cstdint>
#include <utility>

#include "micras/micras.hpp"
#include "micras/states/base.hpp"
#include "micras/states/calibrate_gyroscope.hpp"

namespace micras {
void CalibrateGyroscopeState::on_entry() {
    this->micras.start_gyroscope_calibration();
}

uint8_t CalibrateGyroscopeState::execute() {
    if (this->micras.check_crash()) {
        return std::to_underlying(State::ERROR);
    }

    return this->micras.calibrate_gyroscope() ? std::to_underlying(State::IDLE) : this->get_id();
}
}  // namespace micras
