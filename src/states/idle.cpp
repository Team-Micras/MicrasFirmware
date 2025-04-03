/**
 * @file
 */

#include "micras/states/idle.hpp"

namespace micras {
IdleState::IdleState(uint8_t id, Micras& micras) : State(id), micras{micras} { }

uint8_t IdleState::run() {
    if (this->micras.button_status != proxy::Button::Status::NO_PRESS) {
        this->micras.wait_timer.reset_ms();
        this->micras.wall_sensors.turn_on();
    }

    if (this->micras.button_status == proxy::Button::Status::SHORT_PRESS) {
        this->micras.locomotion.enable();
        this->micras.objective = core::Objective::EXPLORE;
        return Micras::State::WAIT_FOR_RUN;
    }

    if (this->micras.button_status == proxy::Button::Status::LONG_PRESS) {
        this->micras.locomotion.enable();
        this->micras.objective = core::Objective::SOLVE;

        this->micras.maze_storage.sync("maze", this->micras.mapping);

        if (this->micras.dip_switch.get_switch_state(Micras::Switch::DIAGONAL)) {
            this->micras.mapping.diagonalize_best_route();
        }

        if (this->micras.dip_switch.get_switch_state(Micras::Switch::FAN)) {
            this->micras.fan.set_speed(50.0F);
        }

        return Micras::State::WAIT_FOR_RUN;
    }

    if (this->micras.button_status == proxy::Button::Status::EXTRA_LONG_PRESS) {
        return Micras::State::WAIT_FOR_CALIBRATE;
    }

    return this->get_id();
}
}  // namespace micras
