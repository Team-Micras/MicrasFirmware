/**
 * @file
 */

#include <cstdint>

#include "micras/interface.hpp"
#include "micras/micras.hpp"
#include "micras/states/base.hpp"
#include "micras/states/error.hpp"

namespace micras {
void ErrorState::on_entry() {
    this->micras.stop();
    this->micras.send_event(Interface::Event::ERROR);
}

uint8_t ErrorState::execute() {
    return this->get_id();
}
}  // namespace micras
