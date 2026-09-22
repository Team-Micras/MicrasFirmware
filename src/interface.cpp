/**
 * @file
 */

#include <cstdint>
#include <utility>
#include "micras/interface.hpp"
#include "micras/proxy/button.hpp"
#include "micras/proxy/dip_switch.hpp"
#include "micras/proxy/led.hpp"

namespace micras {
Interface::Interface(const proxy::Button& button, const proxy::TDipSwitch<4>& dip_switch, proxy::Led& led) :
    button{button}, dip_switch{dip_switch}, led{led} { }

void Interface::update() {
    if (this->button.get_status() == proxy::Button::Status::SHORT_PRESS) {
        this->send_event(Event::EXPLORE);
    } else if (this->button.get_status() == proxy::Button::Status::LONG_PRESS) {
        this->send_event(Event::SOLVE);
    } else if (this->button.get_status() == proxy::Button::Status::EXTRA_LONG_PRESS) {
        this->send_event(Event::CALIBRATE);
    }

    uint8_t current = 0;

    for (uint8_t i = 0; i < 4; i++) {
        current |= static_cast<uint8_t>(this->dip_switch.get_switch_state(i)) << i;
    }

    if (current != this->profile) {
        this->profile = current;
        this->send_event(Event::PROFILE_MOVED);
    }

    if (this->acknowledge_event(Event::ERROR)) {
        this->led.turn_on();
    }
}

void Interface::send_event(Event event) {
    this->events.at(std::to_underlying(event)) = true;
}

bool Interface::acknowledge_event(Event event) {
    if (this->events.at(std::to_underlying(event))) {
        this->events.at(std::to_underlying(event)) = false;
        return true;
    }

    return false;
}

bool Interface::peek_event(Event event) const {
    return this->events.at(std::to_underlying(event));
}

uint8_t Interface::get_profile() const {
    return this->profile;
}
}  // namespace micras
