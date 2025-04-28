/**
 * @file
 */

#include "micras/micras.hpp"
#include "micras/states/calibrate.hpp"
#include "micras/states/error.hpp"
#include "micras/states/idle.hpp"
#include "micras/states/init.hpp"
#include "micras/states/run.hpp"
#include "micras/states/wait.hpp"
#include "target.hpp"

namespace micras {
Micras::Micras() :
    imu{std::make_shared<proxy::Imu>(imu_config)},
    rotary_sensor_left{std::make_shared<proxy::RotarySensor>(rotary_sensor_left_config)},
    rotary_sensor_right{std::make_shared<proxy::RotarySensor>(rotary_sensor_right_config)},
    wall_sensors{std::make_shared<proxy::WallSensors>(wall_sensors_config)},
    action_queuer{action_queuer_config},
    maze{maze_config},
    odometry{rotary_sensor_left, rotary_sensor_right, imu, odometry_config},
    speed_controller{speed_controller_config},
    follow_wall{wall_sensors, odometry.get_state().pose, follow_wall_config},
    action_pose{odometry.get_state().pose} {
    this->fsm.add_state(std::make_unique<CalibrateState>(State::CALIBRATE, *this));
    this->fsm.add_state(std::make_unique<ErrorState>(State::ERROR, *this));
    this->fsm.add_state(std::make_unique<IdleState>(State::IDLE, *this));
    this->fsm.add_state(std::make_unique<InitState>(State::INIT, *this));
    this->fsm.add_state(std::make_unique<RunState>(State::RUN, *this));
    this->fsm.add_state(std::make_unique<WaitState>(State::WAIT_FOR_RUN, *this, State::RUN));
    this->fsm.add_state(std::make_unique<WaitState>(State::WAIT_FOR_CALIBRATE, *this, State::CALIBRATE));
}

void Micras::update() {
    this->elapsed_time = loop_stopwatch.elapsed_time_us() / 1e6F;
    loop_stopwatch.reset_us();

    this->button.update();
    this->bluetooth.update();
    this->buzzer.update();
    this->fan.update();
    this->imu->update();
    this->wall_sensors->update();
    this->fsm.run();

    while (loop_stopwatch.elapsed_time_us() < loop_time_us) { }
}

bool Micras::calibrate() {
    switch (this->calibration_type) {
        case CalibrationType::SIDE_WALLS:
            this->wall_sensors->calibrate_left_wall();
            this->wall_sensors->calibrate_right_wall();
            this->calibration_type = CalibrationType::FRONT_WALL;
            return false;

        case CalibrationType::FRONT_WALL:
            this->wall_sensors->calibrate_front_wall();
            this->calibration_type = CalibrationType::SIDE_WALLS;
            this->wall_sensors->turn_off();
            return true;
    }

    return false;
}
}  // namespace micras
