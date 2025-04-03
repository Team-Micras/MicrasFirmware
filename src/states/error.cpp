/**
 * @file
 */

#include "micras/states/error.hpp"

namespace micras {
uint8_t ErrorState::run(uint8_t /*previous_state_id*/) {
    this->micras.led.turn_on();

    return this->get_id();
}
}  // namespace micras
