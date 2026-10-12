/**
 * @file
 */

#include <cstdint>
#include <utility>

#include "micras/micras.hpp"
#include "micras/states/base.hpp"
#include "micras/states/calibrate_offsets.hpp"

namespace micras {
void CalibrateOffsetsState::on_entry() {
    this->micras.start_offset_calibration();
}

uint8_t CalibrateOffsetsState::execute() {
    if (this->micras.is_stop_requested() or this->micras.calibrate_offsets()) {
        return std::to_underlying(State::IDLE);
    }

    return this->get_id();
}
}  // namespace micras
