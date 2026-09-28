/**
 * @file
 */

#include <cstdint>
#include <utility>

#include "micras/core/types.hpp"
#include "micras/interface.hpp"
#include "micras/micras.hpp"
#include "micras/states/base.hpp"
#include "micras/states/idle.hpp"

namespace micras {
void IdleState::on_entry() {
    this->micras.stop();
}

uint8_t IdleState::execute() {
    if (this->micras.acknowledge_event(Interface::Event::EXPLORE)) {
        return this->explore();
    }

    if (this->micras.acknowledge_event(Interface::Event::SOLVE)) {
        return this->solve();
    }

    if (this->micras.acknowledge_event(Interface::Event::CALIBRATE)) {
        return this->calibrate();
    }

    return this->get_id();
}

uint8_t IdleState::explore() {
    this->micras.set_objective(core::Objective::EXPLORE);

    return std::to_underlying(State::WAIT_FOR_RUN);
}

uint8_t IdleState::solve() {
    this->micras.set_objective(core::Objective::SOLVE);

    return std::to_underlying(State::PLAN);
}

uint8_t IdleState::calibrate() {
    switch (this->micras.get_maintenance()) {
        case Micras::Maintenance::WALL_SENSORS:
            return std::to_underlying(State::WAIT_FOR_CALIBRATE);

        case Micras::Maintenance::DRIVE:
            return std::to_underlying(State::WAIT_FOR_IDENTIFY);

        case Micras::Maintenance::GYROSCOPE:
            return std::to_underlying(State::WAIT_FOR_GYROSCOPE);
    }

    return this->get_id();
}
}  // namespace micras
