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
    logger{std::make_shared<comm::Logger>(debug_mode)},
    pool{std::make_shared<comm::SerialVariablePool>()},
    comm_service{comm::CommunicationService(pool, logger)},
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
    follow_wall{wall_sensors, follow_wall_config},
    interface{pool, argb, button, buzzer, dip_switch, led},
    action_pose{odometry.get_state().pose} {
    this->fsm.add_state(std::make_unique<CalibrateState>(State::CALIBRATE, *this));
    this->fsm.add_state(std::make_unique<ErrorState>(State::ERROR, *this));
    this->fsm.add_state(std::make_unique<IdleState>(State::IDLE, *this));
    this->fsm.add_state(std::make_unique<InitState>(State::INIT, *this));
    this->fsm.add_state(std::make_unique<RunState>(State::RUN, *this));
    this->fsm.add_state(std::make_unique<WaitState>(State::WAIT_FOR_RUN, *this, State::RUN));
    this->fsm.add_state(std::make_unique<WaitState>(State::WAIT_FOR_CALIBRATE, *this, State::CALIBRATE));

    this->comm_service.register_communication_functions(
        [this](const std::vector<uint8_t>& data) { this->bluetooth.send_data(data); },
        [this]() { return this->bluetooth.get_data(); }
    );

    this->pool->add_variable("Desired Linear Speed", this->desired_speeds.linear);
    this->pool->add_variable("Desired Angular Speed", this->desired_speeds.angular);
    this->pool->add_variable("Linear PID Response", this->last_pid_response.linear);
    this->pool->add_variable("Angular PID Response", this->last_pid_response.angular);
    this->pool->add_variable("Left Feed Forward Response", this->left_ff);
    this->pool->add_variable("Right Feed Forward Response", this->right_ff);
    this->pool->add_variable("Wall Sensors 0", this->wall_sensor_reading[wall_sensors_index.left_front]);
    this->pool->add_variable("Wall Sensors 1", this->wall_sensor_reading[wall_sensors_index.left]);
    this->pool->add_variable("Wall Sensors 2", this->wall_sensor_reading[wall_sensors_index.right]);
    this->pool->add_variable("Wall Sensors 3", this->wall_sensor_reading[wall_sensors_index.right_front]);
    this->pool->add_variable("Rotary Sensor Left", this->rotary_sensor_left_reading);
    this->pool->add_variable("Rotary Sensor Right", this->rotary_sensor_right_reading);
    this->pool->add_variable("Loop Time", this->elapsed_time);
    // this->pool->add_variable("Odometry State", odometry.get_state());  // @TODO implementar no app
    // this->pool->add_variable("Grid Pose", this->grid_pose);            // @TODO implementar no app
    this->pool->add_variable("Odometry Linear Velocity", odometry.get_state().velocity.linear);
    this->pool->add_variable("Odometry Angular Velocity", odometry.get_state().velocity.angular);
}

void Micras::update() {
    this->elapsed_time = loop_stopwatch.elapsed_time_us() / 1e6F;
    loop_stopwatch.reset_us();

    this->update_monitoring_variables();

    this->button->update();
    this->buzzer->update();
    this->bluetooth.update();
    this->interface.update();
    this->comm_service.update();

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
        this->grid_pose = this->maze.get_next_goal(this->grid_pose, false);
        this->action_queuer.recompute({});
        this->current_action = this->action_queuer.pop();
    } else {
        this->current_action = this->action_queuer.pop();
    }
}

bool Micras::run() {
    this->odometry.update(this->elapsed_time);

    micras::nav::State& state = this->odometry.get_state();
    core::Observation   observation{};

    if (this->current_action->finished(this->action_pose.get())) {
        if (this->finished) {
            this->finished = false;
            this->locomotion.stop();

            if (this->objective != core::Objective::SOLVE) {
                this->maze.compute_best_route();
            }

            return true;
        }

        this->speed_controller.reset();
        this->action_pose.reset_reference();

        if (not this->action_queuer.empty()) {
            this->current_action = this->action_queuer.pop();
        } else {
            const bool returning = (this->objective == core::Objective::RETURN);
            const bool solving = (this->objective == core::Objective::SOLVE);

            if (not solving) {
                observation = this->follow_wall.get_observation();
                this->maze.update_walls(this->grid_pose, observation);
            }

            micras::nav::GridPose next_goal{};

            if (solving or this->maze.finished(this->grid_pose.position, returning)) {
                this->finished = true;
                next_goal = this->grid_pose.turned_back().front();
            } else {
                next_goal = this->maze.get_next_goal(this->grid_pose, returning);
            }

            this->action_queuer.push_exploring(this->grid_pose, next_goal.position);
            this->current_action = this->action_queuer.pop();
            this->grid_pose = next_goal;
        }
    }

    this->desired_speeds = this->current_action->get_speeds(this->action_pose.get(), this->elapsed_time);

    if (this->current_action->allow_follow_wall()) {
        this->desired_speeds.angular = this->follow_wall.compute_angular_correction(this->elapsed_time, state);
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
    this->imu->calibrate();
    this->action_pose.reset_reference();
    this->fan.set_speed(fan_speed);
}

void Micras::reset() {
    this->grid_pose = maze_config.start;
    this->odometry.reset();
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
    this->action_queuer.recompute(this->maze.get_best_route(), false);
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

void Micras::handle_events() {
    if (this->interface.acknowledge_event(Interface::Event::TURN_ON_FAN)) {
        this->fan.enable();
    } else if (this->interface.acknowledge_event(Interface::Event::TURN_OFF_FAN)) {
        this->fan.disable();
    }
}

void Micras::update_monitoring_variables() {
    this->last_pid_response = this->speed_controller.get_last_pid_response();
    this->wall_sensor_reading[0] = this->wall_sensors->get_reading(wall_sensors_index.left_front);
    this->wall_sensor_reading[1] = this->wall_sensors->get_reading(wall_sensors_index.left);
    this->wall_sensor_reading[2] = this->wall_sensors->get_reading(wall_sensors_index.right);
    this->wall_sensor_reading[3] = this->wall_sensors->get_reading(wall_sensors_index.right_front);
    this->rotary_sensor_left_reading = this->rotary_sensor_left->get_position();
    this->rotary_sensor_right_reading = this->rotary_sensor_right->get_position();
}
}  // namespace micras
