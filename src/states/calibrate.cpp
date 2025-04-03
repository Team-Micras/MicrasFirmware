/**
 * @file
 */

#include "micras/states/calibrate.hpp"

namespace micras {
uint8_t CalibrateState::run(uint8_t /*previous_state_id*/) {
    if (this->micras.calibration_type == Micras::CalibrationType::RIGHT_FREE_SPACE) {
        return Micras::State::IDLE;
    }

    this->calibrate();
    return Micras::State::WAIT_FOR_CALIBRATE;
}

void CalibrateState::calibrate() {
    switch (this->micras.calibration_type) {
        case Micras::CalibrationType::SIDE_WALLS:
            this->micras.go_to_point.calibrate();
            this->micras.mapping.calibrate_side();
            this->micras.wall_sensors.calibrate_left_wall();
            this->micras.wall_sensors.calibrate_right_wall();
            this->micras.wall_sensors.calibrate_front_free_space();
            this->micras.calibration_type = Micras::CalibrationType::FRONT_WALL;
            break;

        case Micras::CalibrationType::FRONT_WALL:
            this->micras.mapping.calibrate_front();
            this->micras.wall_sensors.calibrate_front_wall();
            this->micras.calibration_type = Micras::CalibrationType::LEFT_FREE_SPACE;
            break;

        case Micras::CalibrationType::LEFT_FREE_SPACE:
            this->micras.wall_sensors.calibrate_left_free_space();
            this->micras.calibration_type = Micras::CalibrationType::RIGHT_FREE_SPACE;
            break;

        case Micras::CalibrationType::RIGHT_FREE_SPACE:
            this->micras.wall_sensors.calibrate_right_free_space();
            this->micras.wall_sensors.update_thresholds();
            this->micras.wall_sensors.turn_off();
            this->micras.calibration_type = Micras::CalibrationType::SIDE_WALLS;
            break;
    }
}
}  // namespace micras
