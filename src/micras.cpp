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
    loop_timer{timer_config},
    argb{argb_config},
    battery{battery_config},
    button{button_config},
    buzzer{buzzer_config},
    dip_switch{dip_switch_config},
    wall_sensors{wall_sensors_config},
    fan{fan_config},
    imu{imu_config},
    led{led_config},
    locomotion{locomotion_config},
    rotary_sensor_left{rotary_sensor_left_config},
    rotary_sensor_right{rotary_sensor_right_config},
    // torque_sensors{torque_sensors_config},
    maze_storage{maze_storage_config},
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
    this->elapsed_time = loop_timer.elapsed_time_us() / 1e6F;
    loop_timer.reset_us();

    this->button_status = button.get_status();

    this->wall_sensors.update();
    this->buzzer.update();
    this->imu.update();
    this->fan.update();
    this->fsm.run();

    while (loop_timer.elapsed_time_us() < loop_time_us) { }
}
}  // namespace micras
