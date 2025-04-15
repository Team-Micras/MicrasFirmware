/**
 * @file
 */

#ifndef CALIBRATE_STATE_HPP
#define CALIBRATE_STATE_HPP

#include "micras/states/base.hpp"

namespace micras {
class CalibrateState : public BaseState {
public:
    using BaseState::BaseState;

    /**
     * @brief Execute this state.
     *
     * @param previous_state_id The id of the last executed state.
     *
     * @return The id of the next state.
     */
    uint8_t run(uint8_t /*previous_state_id*/) override {
        this->calibrate();

        if (this->micras.calibration_type == Micras::CalibrationType::SIDE_WALLS) {
            return Micras::State::IDLE;
        }

        return Micras::State::WAIT_FOR_CALIBRATE;
    }

private:
    /**
     * @brief Calibrate the robot.
     */
    void calibrate() {
        switch (this->micras.calibration_type) {
            case Micras::CalibrationType::SIDE_WALLS:
                this->micras.wall_sensors->calibrate_left_wall();
                this->micras.wall_sensors->calibrate_right_wall();
                this->micras.calibration_type = Micras::CalibrationType::FRONT_WALL;
                break;

            case Micras::CalibrationType::FRONT_WALL:
                this->micras.wall_sensors->calibrate_front_wall();
                this->micras.calibration_type = Micras::CalibrationType::SIDE_WALLS;
                this->micras.wall_sensors->turn_off();
                break;
        }
    }
};
}  // namespace micras

#endif  // CALIBRATE_STATE_HPP
