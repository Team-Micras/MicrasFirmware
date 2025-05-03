/**
 * @file
 */

#include "micras/interface.hpp"

namespace micras {
Interface::Interface(
    const std::shared_ptr<proxy::TArgb<2>>& Argb, const std::shared_ptr<proxy::Button>& button,
    const std::shared_ptr<proxy::Buzzer>& buzzer, const std::shared_ptr<proxy::TDipSwitch<4>>& dip_switch,
    const std::shared_ptr<proxy::Led>& led
) :
    argb{Argb}, button{button}, buzzer{buzzer}, dip_switch{dip_switch}, led{led} { }

void Interface::update() {
    if (this->button->get_status() == proxy::Button::Status::SHORT_PRESS) {
        this->send_event(Event::EXPLORE);
    } else if (this->button->get_status() == proxy::Button::Status::LONG_PRESS) {
        this->send_event(Event::SOLVE);
    } else if (this->button->get_status() == proxy::Button::Status::EXTRA_LONG_PRESS) {
        this->send_event(Event::CALIBRATE);
    }

    if (this->acknowledge_event(Event::ERROR)) {
        this->led->turn_on();
    }
}

void Interface::send_event(Event event) {
    this->events.at(event) = true;
}

bool Interface::acknowledge_event(Event event) {
    if (this->events.at(event)) {
        this->events.at(event) = false;
        return true;
    }

    return false;
}

bool Interface::peek_event(Event event) const {
    return this->events.at(event);
}
}  // namespace micras
