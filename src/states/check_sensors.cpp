/**
 * @file
 */

#include <cstdint>
#include <utility>

#include "micras/micras.hpp"
#include "micras/states/base.hpp"
#include "micras/states/check_sensors.hpp"

namespace micras {
void CheckSensorsState::on_entry() {
    this->micras.start_sensor_check();
}

uint8_t CheckSensorsState::execute() {
    this->micras.rest();

    return this->micras.is_stop_requested() ? std::to_underlying(State::IDLE) : this->get_id();
}
}  // namespace micras
