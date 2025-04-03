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
    CalibrateState calibrate_state{State::CALIBRATE, *this};
    ErrorState     error_state{State::ERROR, *this};
    IdleState      idle_state{State::IDLE, *this};
    InitState      init_state{State::INIT, *this};
    RunState       run_state{State::RUN, *this};
    WaitState      wait_for_run_state{State::WAIT_FOR_RUN, *this, State::RUN};
    WaitState      wait_for_calibrate_state{State::WAIT_FOR_CALIBRATE, *this, State::CALIBRATE};

    this->fsm.add_state(calibrate_state);
    this->fsm.add_state(error_state);
    this->fsm.add_state(idle_state);
    this->fsm.add_state(init_state);
    this->fsm.add_state(run_state);
    this->fsm.add_state(wait_for_run_state);
    this->fsm.add_state(wait_for_calibrate_state);
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
