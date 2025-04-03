/**
 * @file
 */

#include "micras/states/error.hpp"

namespace micras {
ErrorState::ErrorState(uint8_t id, Micras& micras) : State(id), micras{micras} { }

uint8_t ErrorState::run() {
    this->micras.led.turn_on();

    return this->get_id();
}
}  // namespace micras
