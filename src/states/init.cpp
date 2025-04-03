/**
 * @file
 */

#include "micras/states/init.hpp"

namespace micras {
InitState::InitState(uint8_t id, Micras& micras) : State(id), micras{micras} { }

uint8_t InitState::run() {
    if (not this->micras.imu.check_whoami()) {
        return Micras::State::ERROR;
    }

    return Micras::State::IDLE;
}
}  // namespace micras
