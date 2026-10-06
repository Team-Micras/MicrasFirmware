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
        this->micras.set_objective(core::Objective::EXPLORE);

        return std::to_underlying(State::WAIT_FOR_RUN);
    }

    if (this->micras.acknowledge_event(Interface::Event::SOLVE)) {
        this->micras.set_objective(core::Objective::SOLVE);

        return std::to_underlying(State::PLAN);
    }

    if (this->micras.acknowledge_event(Interface::Event::CALIBRATE)) {
        switch (this->micras.take_maintenance()) {
            case Micras::Maintenance::WALL_SENSORS:
                return std::to_underlying(State::WAIT_FOR_CALIBRATE);

            case Micras::Maintenance::DRIVE:
                return std::to_underlying(State::WAIT_FOR_IDENTIFY);

            case Micras::Maintenance::GYROSCOPE:
                return std::to_underlying(State::WAIT_FOR_GYROSCOPE);

            case Micras::Maintenance::SENSORS:
                return std::to_underlying(State::CHECK_SENSORS);

            case Micras::Maintenance::POLARITY:
                return std::to_underlying(State::CHECK_POLARITY);

            case Micras::Maintenance::CROSSTALK:
                return std::to_underlying(State::CHECK_CROSSTALK);

            case Micras::Maintenance::WALL_OFFSETS:
                return std::to_underlying(State::CALIBRATE_OFFSETS);

            case Micras::Maintenance::NUMBER_OF_PROCEDURES:
                break;
        }
    }

    return this->get_id();
}
}  // namespace micras
