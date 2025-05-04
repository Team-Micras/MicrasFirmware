/**
 * @file
 */

#include <tuple>

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
    argb{std::make_shared<proxy::Argb>(argb_config)},
    button{std::make_shared<proxy::Button>(button_config)},
    buzzer{std::make_shared<proxy::Buzzer>(buzzer_config)},
    dip_switch{std::make_shared<proxy::DipSwitch>(dip_switch_config)},
    led{std::make_shared<proxy::Led>(led_config)},
    imu{std::make_shared<proxy::Imu>(imu_config)},
    rotary_sensor_left{std::make_shared<proxy::RotarySensor>(rotary_sensor_left_config)},
    rotary_sensor_right{std::make_shared<proxy::RotarySensor>(rotary_sensor_right_config)},
    wall_sensors{std::make_shared<proxy::WallSensors>(wall_sensors_config)},
    action_queuer{action_queuer_config},
    maze{maze_config},
    odometry{rotary_sensor_left, rotary_sensor_right, imu, odometry_config},
    speed_controller{speed_controller_config},
    follow_wall{wall_sensors, odometry.get_state().pose, follow_wall_config},
    interface{argb, button, buzzer, dip_switch, led},
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

    this->button->update();
    this->buzzer->update();
    this->interface.update();

    this->fan.update();
    this->imu->update();
    this->wall_sensors->update();

    this->fsm.update();

    while (loop_stopwatch.elapsed_time_us() < loop_time_us) { }
}

bool Micras::calibrate() {
    switch (this->calibration_type) {
        case CalibrationType::SIDE_WALLS:
            this->wall_sensors->calibrate_sensor(wall_sensors_index.left);
            this->wall_sensors->calibrate_sensor(wall_sensors_index.right);
            this->calibration_type = CalibrationType::FRONT_WALL;
            return false;

        case CalibrationType::FRONT_WALL:
            this->wall_sensors->calibrate_sensor(wall_sensors_index.left_front);
            this->wall_sensors->calibrate_sensor(wall_sensors_index.right_front);
            this->calibration_type = CalibrationType::SIDE_WALLS;
            this->wall_sensors->turn_off();
            return true;
    }

    return false;
}

void Micras::prepare() {
    if (this->objective == core::Objective::EXPLORE) {
        this->grid_pose = this->maze.get_next_goal(this->grid_pose.position);
        this->action_queuer.recompute({});
        this->current_action = this->action_queuer.pop();
    } else {
        this->current_action = this->action_queuer.pop();
    }
}

bool Micras::run() {
    this->odometry.update(this->elapsed_time);

    const micras::nav::State& state = this->odometry.get_state();
    core::Observation         observation{};

    if (this->current_action->finished(this->action_pose.get())) {
        if (this->finished) {
            this->finished = false;
            this->locomotion.stop();

            if (this->objective == core::Objective::EXPLORE) {
                this->maze.compute_return_costmap();
            }

            return true;
        }

        this->speed_controller.reset();
        this->action_pose.reset_reference();

        if (not this->action_queuer.empty()) {
            this->current_action = this->action_queuer.pop();
        } else {
            const bool solving = (this->objective == core::Objective::SOLVE);

            if (not solving) {
                observation = this->follow_wall.get_observation();
                this->maze.update_walls(this->grid_pose, observation);
            }

            micras::nav::GridPose next_goal{};

            if (solving or this->maze.finished(this->grid_pose.position)) {
                this->finished = true;
                next_goal = this->grid_pose.turned_back().front();
            } else {
                next_goal = this->maze.get_next_goal(this->grid_pose.position);
            }

            this->action_queuer.push(this->grid_pose, next_goal.position);
            this->current_action = this->action_queuer.pop();
            this->grid_pose = next_goal;
        }

        if (this->current_action->allow_follow_wall()) {
            this->follow_wall.reset();
        }
    }

    this->desired_speeds = this->current_action->get_speeds(this->action_pose.get());

    if (this->current_action->allow_follow_wall()) {
        this->desired_speeds.angular =
            this->follow_wall.compute_angular_correction(this->elapsed_time, state.velocity.linear);
    }

    std::tie(this->left_response, this->right_response) =
        this->speed_controller.compute_control_commands(state.velocity, desired_speeds, this->elapsed_time);

    std::tie(this->left_ff, this->right_ff) =
        this->speed_controller.compute_feed_forward_commands(desired_speeds, this->elapsed_time);

    this->locomotion.set_wheel_command(this->left_ff + this->left_response, this->right_ff + this->right_response);

    return false;
}

void Micras::stop() {
    this->wall_sensors->turn_off();
    this->locomotion.stop();
    this->locomotion.disable();
    this->fan.stop();
}

void Micras::init() {
    this->wall_sensors->turn_on();
    this->locomotion.enable();
    this->odometry.reset();
    this->imu->calibrate();
    this->action_pose.reset_reference();
}

void Micras::reset() {
    this->grid_pose = maze_config.start;
    this->finished = false;
}

bool Micras::check_crash() const {
    return std::hypot(
               this->imu->get_linear_acceleration(proxy::Imu::Axis::X),
               this->imu->get_linear_acceleration(proxy::Imu::Axis::Y)
           ) > crash_acceleration;
}

void Micras::save_best_route() {
    this->maze_storage.create("maze", this->maze);
    this->maze_storage.save();
}

void Micras::load_best_route() {
    this->maze_storage.sync("maze", this->maze);
    this->action_queuer.recompute(this->maze.get_best_route());
}

core::Objective Micras::get_objective() const {
    return this->objective;
}

void Micras::set_objective(core::Objective objective) {
    this->objective = objective;
}

bool Micras::check_initialization() const {
    return this->imu->was_initialized();
}

void Micras::send_event(Interface::Event event) {
    this->interface.send_event(event);
}

bool Micras::acknowledge_event(Interface::Event event) {
    return this->interface.acknowledge_event(event);
}

bool Micras::peek_event(Interface::Event event) const {
    return this->interface.peek_event(event);
}
}  // namespace micras
