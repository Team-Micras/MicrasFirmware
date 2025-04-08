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
    odometry{rotary_sensor_left, rotary_sensor_right, imu, odometry_config},
    mapping{wall_sensors, mapping_config},
    look_at_point{look_at_point_config},
    go_to_point{wall_sensors, go_to_point_config, follow_wall_config} {
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

    this->button_status = button.get_status();

    this->buzzer.update();
    this->fan.update();
    this->imu->update();
    this->wall_sensors->update();
    this->fsm.run();

    while (loop_stopwatch.elapsed_time_us() < loop_time_us) { }
}
}  // namespace micras
