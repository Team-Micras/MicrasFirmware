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
    this->pool->add_variable("Explore", this->comm_explore, comm::SerialVariablePool::VarType::BIDIRECTIONAL);
    this->pool->add_variable("Solve", this->comm_solve, comm::SerialVariablePool::VarType::BIDIRECTIONAL);
    this->pool->add_variable("Calibrate", this->comm_calibrate, comm::SerialVariablePool::VarType::BIDIRECTIONAL);
    this->pool->add_variable("Fan", this->comm_fan, comm::SerialVariablePool::VarType::BIDIRECTIONAL);
    this->pool->add_variable("Diagonal", this->comm_diagonal, comm::SerialVariablePool::VarType::BIDIRECTIONAL);
    this->pool->add_variable("Boost", this->comm_boost, comm::SerialVariablePool::VarType::BIDIRECTIONAL);
    this->pool->add_variable("Risky", this->comm_risky, comm::SerialVariablePool::VarType::BIDIRECTIONAL);
}

void Interface::update() {
    for (const auto& condition : this->event_conditions) {
        bool result = (this->*condition.check)();

        if (condition.type == ConditionType::Trigger) {
            if (result)
                send_event(condition.true_event);
        } else {
            send_event(result ? condition.true_event : condition.false_event);
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
