/**
 * @file
 */

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <span>
#include <tuple>
#include <utility>

#include "constants.hpp"
#include "micras/comm/link.hpp"
#include "micras/core/types.hpp"
#include "micras/hal/mcu.hpp"
#include "micras/micras.hpp"
#include "micras/proxy/button.hpp"
#include "micras/proxy/buzzer.hpp"
#include "micras/proxy/imu.hpp"
#include "micras/states/calibrate.hpp"
#include "micras/states/error.hpp"
#include "micras/states/idle.hpp"
#include "micras/states/init.hpp"
#include "micras/states/run.hpp"
#include "micras/states/wait.hpp"
#include "target.hpp"

namespace micras {
// NOLINTBEGIN(cppcoreguidelines-avoid-non-const-global-variables) the DMA and the capture write here
static std::array<uint8_t, bluetooth_rx_buffer_size> bluetooth_rx_buffer;
static std::array<uint8_t, bluetooth_tx_buffer_size> bluetooth_tx_buffer;
static std::array<uint8_t, trace_buffer_size>        trace_buffer;

// NOLINTEND(cppcoreguidelines-avoid-non-const-global-variables)

Micras::Micras() :
    bluetooth{bluetooth_config, bluetooth_rx_buffer, bluetooth_tx_buffer},
    action_queuer{action_queuer_config},
    maze{maze_config},
    odometry{rotary_sensor_left, rotary_sensor_right, imu, odometry_config},
    speed_controller{speed_controller_config},
    follow_wall{wall_sensors, follow_wall_config},
    trace{variables, trace_buffer},
    link{bluetooth, variables, trace, *this, {.loop_time_us = loop_time_us}},
    interface{button, dip_switch, led},
    action_pose{odometry.get_state().pose} {
    hal::Mcu::set_watchdog_timeout(watchdog_timeout_ms);

    this->fsm.add_state(std::make_unique<CalibrateState>(State::CALIBRATE, *this));
    this->fsm.add_state(std::make_unique<ErrorState>(State::ERROR, *this));
    this->fsm.add_state(std::make_unique<IdleState>(State::IDLE, *this));
    this->fsm.add_state(std::make_unique<InitState>(State::INIT, *this));
    this->fsm.add_state(std::make_unique<RunState>(State::RUN, *this));
    this->fsm.add_state(std::make_unique<WaitState>(State::WAIT_FOR_RUN, *this, State::RUN));
    this->fsm.add_state(std::make_unique<WaitState>(State::WAIT_FOR_CALIBRATE, *this, State::CALIBRATE));

    this->register_variables();
}

void Micras::register_variables() {
    this->imu.register_variables(this->variables, "imu/");
    this->wall_sensors.register_variables(this->variables, "wall/");

    this->variables.add("loop/", "elapsed_time", this->elapsed_time, {.stream = true});
    this->variables.add("loop/", "worst_time_us", this->worst_loop_time_us, {.stream = true});

    this->variables.add("cmd/", "linear", this->desired_speeds.linear, {.stream = true});
    this->variables.add("cmd/", "angular", this->desired_speeds.angular, {.stream = true});

    this->variables.add("response/", "left", this->left_response, {.stream = true});
    this->variables.add("response/", "right", this->right_response, {.stream = true});
    this->variables.add("feed_forward/", "left", this->left_ff, {.stream = true});
    this->variables.add("feed_forward/", "right", this->right_ff, {.stream = true});

    this->variables.add("", "objective", this->objective, {.stream = true, .write = true, .idle = true});
    this->variables.add("", "run_profile", this->run_profile, {.stream = true, .write = true, .persist = true});
    this->variables.add("", "maze", this->maze, {.persist = true});

    this->link.register_variables(this->variables, "link/");

    this->maze_storage.restore(this->variables);
}

void Micras::update() {
    this->elapsed_time = static_cast<float>(this->loop_stopwatch.elapsed_time_us()) / 1e6F;
    this->loop_stopwatch.reset_us();

    hal::Mcu::refresh_watchdog();

    this->button.update();
    this->buzzer.update();
    this->interface.update();

    this->battery.update();
    this->fan.update();
    this->imu.update();
    this->torque_sensors.update();
    this->wall_sensors.update();
    this->bluetooth.update();

    this->fsm.update();

    const uint32_t timestamp_us = this->telemetry_stopwatch.elapsed_time_us();

    this->link.poll(this->is_idle());
    this->trace.sample(timestamp_us);
    this->link.pump(timestamp_us);

    this->worst_loop_time_us = std::max(this->worst_loop_time_us, this->loop_stopwatch.elapsed_time_us());

    while (this->loop_stopwatch.elapsed_time_us() < loop_time_us) { }
}

bool Micras::calibrate() {
    switch (this->calibration_type) {
        case CalibrationType::SIDE_WALLS:
            this->wall_sensors.calibrate_sensor(wall_sensors_index.left);
            this->wall_sensors.calibrate_sensor(wall_sensors_index.right);
            this->calibration_type = CalibrationType::FRONT_WALL;
            return false;

        case CalibrationType::FRONT_WALL:
            this->wall_sensors.calibrate_sensor(wall_sensors_index.left_front);
            this->wall_sensors.calibrate_sensor(wall_sensors_index.right_front);
            this->calibration_type = CalibrationType::SIDE_WALLS;
            this->wall_sensors.turn_off();
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
    this->wall_sensors.turn_off();
    this->locomotion.stop();
    this->locomotion.disable();
    this->fan.stop();
}

void Micras::init() {
    this->wall_sensors.turn_on();
    this->locomotion.enable();
    this->imu.calibrate();
    this->action_pose.reset_reference();
}

void Micras::reset() {
    this->grid_pose = maze_config.start;
    this->odometry.reset();
    this->finished = false;
}

bool Micras::check_crash() const {
    return std::hypot(
               this->imu.get_linear_acceleration(proxy::Imu::Axis::X),
               this->imu.get_linear_acceleration(proxy::Imu::Axis::Y)
           ) > crash_acceleration;
}

void Micras::save_best_route() {
    hal::Mcu::set_watchdog_timeout(flash_watchdog_timeout_ms);

    this->maze_storage.save(this->variables);

    hal::Mcu::set_watchdog_timeout(watchdog_timeout_ms);
}

void Micras::load_best_route() {
    this->action_queuer.recompute(this->maze.get_best_route(), false);
    this->fan.set_speed(fan_speed);
}

core::Objective Micras::get_objective() const {
    return this->objective;
}

void Micras::set_objective(core::Objective objective) {
    this->objective = objective;
}

bool Micras::check_initialization() const {
    return this->imu.was_initialized();
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
    if (this->interface.acknowledge_event(Interface::Event::PROFILE_MOVED)) {
        this->run_profile = this->interface.get_profile();
    }

    if ((this->run_profile & std::to_underlying(Interface::Profile::FAN)) != 0) {
        this->fan.enable();
    } else {
        this->fan.disable();
    }
}

bool Micras::is_idle() const {
    return this->fsm.get_current_state_id() == std::to_underlying(State::IDLE);
}

comm::CommandResult Micras::handle_command(uint8_t code, uint32_t argument) {
    switch (static_cast<Command>(code)) {
        case Command::EXPLORE:
            this->send_event(Interface::Event::EXPLORE);
            return comm::CommandResult::OK;

        case Command::SOLVE:
            this->send_event(Interface::Event::SOLVE);
            return comm::CommandResult::OK;

        case Command::CALIBRATE:
            this->send_event(Interface::Event::CALIBRATE);
            return comm::CommandResult::OK;

        case Command::TRACE_TRIGGER:
            this->trace.fire();
            return comm::CommandResult::OK;

        case Command::SAVE:
            if (not this->is_idle()) {
                return comm::CommandResult::REFUSED;
            }

            this->save_best_route();
            return comm::CommandResult::OK;

        case Command::RESET:
            if (not this->is_idle()) {
                return comm::CommandResult::REFUSED;
            }

            this->reset();
            return comm::CommandResult::OK;
    }

    static_cast<void>(argument);
    return comm::CommandResult::UNKNOWN;
}
}  // namespace micras
