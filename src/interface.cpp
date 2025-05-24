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

    for (uint8_t i = 0; i < 4; i++) {
        if (this->dip_switch->get_switch_state(i) == this->dip_switch_states.at(i)) {
            continue;
        }

        this->dip_switch_states.at(i) = this->dip_switch->get_switch_state(i);

        switch (i) {
            case DipSwitchPins::FAN:
                this->send_event(this->dip_switch_states.at(i) ? Event::TURN_ON_FAN : Event::TURN_OFF_FAN);
                break;
            case DipSwitchPins::DIAGONAL:
                this->send_event(this->dip_switch_states.at(i) ? Event::TURN_ON_DIAGONAL : Event::TURN_OFF_DIAGONAL);
                break;
            case DipSwitchPins::BOOST:
                this->send_event(this->dip_switch_states.at(i) ? Event::TURN_ON_BOOST : Event::TURN_OFF_BOOST);
                break;
            case DipSwitchPins::RISKY:
                this->send_event(this->dip_switch_states.at(i) ? Event::TURN_ON_RISKY : Event::TURN_OFF_RISKY);
                break;
        }
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
