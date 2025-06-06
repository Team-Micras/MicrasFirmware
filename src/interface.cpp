/**
 * @file
 */

#include "micras/interface.hpp"

namespace micras {
Interface::Interface(
    const std::shared_ptr<comm::SerialVariablePool>& pool, const std::shared_ptr<proxy::TArgb<2>>& argb,
    const std::shared_ptr<proxy::Button>& button, const std::shared_ptr<proxy::Buzzer>& buzzer,
    const std::shared_ptr<proxy::TDipSwitch<4>>& dip_switch, const std::shared_ptr<proxy::Led>& led
) :
    pool{pool}, argb{argb}, button{button}, buzzer{buzzer}, dip_switch{dip_switch}, led{led} {
    this->pool->add_write_only("Explore", this->comm_explore);
    this->pool->add_write_only("Solve", this->comm_solve);
    this->pool->add_write_only("Calibrate", this->comm_calibrate);
    this->pool->add_write_only("Fan", this->comm_fan);
    this->pool->add_write_only("Diagonal", this->comm_diagonal);
    this->pool->add_write_only("Boost", this->comm_boost);
    this->pool->add_write_only("Risky", this->comm_risky);
}

void Interface::update() {
    for (const auto& cond : event_conditions) {
        bool result = (this->*cond.check)();

        if (cond.type == ConditionType::Trigger) {
            if (result)
                send_event(cond.true_event);
        } else {
            send_event(result ? cond.true_event : cond.false_event);
        }
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

bool Interface::condition_explore() const {
    return this->button->get_status() == proxy::Button::Status::SHORT_PRESS || comm_explore;
}

bool Interface::condition_solve() const {
    return this->button->get_status() == proxy::Button::Status::LONG_PRESS || comm_solve;
}

bool Interface::condition_calibrate() const {
    return this->button->get_status() == proxy::Button::Status::EXTRA_LONG_PRESS || comm_calibrate;
}

bool Interface::condition_fan() const {
    return this->dip_switch->get_switch_state(DipSwitchPins::FAN) || this->comm_fan;
}

bool Interface::condition_diagonal() const {
    return this->dip_switch->get_switch_state(DipSwitchPins::DIAGONAL) || this->comm_diagonal;
}

bool Interface::condition_boost() const {
    return this->dip_switch->get_switch_state(DipSwitchPins::BOOST) || this->comm_boost;
}

bool Interface::condition_risky() const {
    return this->dip_switch->get_switch_state(DipSwitchPins::RISKY) || this->comm_risky;
}
}  // namespace micras
