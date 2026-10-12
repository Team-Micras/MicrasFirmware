/**
 * @file
 */

#include <cstdint>
#include <utility>

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
    if (this->micras.acknowledge_event(Interface::Event::RESUME) and this->micras.check_initialization()) {
        this->micras.leave_error();
        return std::to_underlying(State::IDLE);
    }

    return this->get_id();
}
}  // namespace micras
