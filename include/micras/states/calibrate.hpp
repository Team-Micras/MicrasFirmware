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
};
}  // namespace micras

#endif  // CALIBRATE_STATE_HPP
