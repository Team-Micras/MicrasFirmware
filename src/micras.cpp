/**
 * @file
 */

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <string_view>
#include <utility>

#include "constants.hpp"
#include "micras/comm/link.hpp"
#include "micras/core/types.hpp"
#include "micras/interface.hpp"
#include "micras/micras.hpp"
#include "micras/nav/controller.hpp"
#include "micras/nav/localizer.hpp"
#include "micras/nav/measurements.hpp"
#include "micras/nav/motion_limits.hpp"
#include "micras/nav/robot_model.hpp"
#include "micras/nav/segment.hpp"
#include "micras/nav/state.hpp"
#include "micras/proxy/imu.hpp"
#include "micras/proxy/locomotion.hpp"
#include "micras/states/base.hpp"
#include "target.hpp"

// NOLINTBEGIN(cppcoreguidelines-avoid-non-const-global-variables)
static volatile float    monitor_pose_x;
static volatile float    monitor_pose_y;
static volatile float    monitor_pose_orientation;
static volatile float    monitor_linear_speed;
static volatile float    monitor_angular_speed;
static volatile float    monitor_gyroscope_bias;
static volatile float    monitor_position_deviation;
static volatile float    monitor_orientation_deviation;
static volatile float    monitor_innovation_level;
static volatile uint32_t monitor_corrections_accepted;
static volatile uint32_t monitor_corrections_rejected;
static volatile uint32_t monitor_edges_used;

static volatile float monitor_reference_x;
static volatile float monitor_reference_y;
static volatile float monitor_reference_orientation;
static volatile float monitor_reference_linear_speed;
static volatile float monitor_reference_angular_speed;
static volatile float monitor_along_error;
static volatile float monitor_across_error;
static volatile float monitor_orientation_error;
static volatile float monitor_forward_feed_forward;
static volatile float monitor_rotation_feed_forward;
static volatile float monitor_forward_feedback;
static volatile float monitor_rotation_feedback;

// NOLINTBEGIN(*-avoid-c-arrays) a volatile std::array cannot be written to
static volatile float monitor_wall_distances[4];
static volatile float monitor_wall_reference_readings[4];
static volatile float monitor_wall_calibration_spreads[4];
// NOLINTEND(*-avoid-c-arrays)

static volatile uint32_t monitor_worst_loop_time_us;
static volatile uint32_t monitor_missed_ticks;
static volatile uint32_t monitor_saturated_iterations;
static volatile float    monitor_route_time;

static volatile bool  monitor_identification_valid;
static volatile float monitor_breakaway_voltage;
static volatile float monitor_linear_static_friction;
static volatile float monitor_linear_speed_constant;
static volatile float monitor_linear_acceleration_constant;
static volatile float monitor_angular_static_friction;
static volatile float monitor_angular_speed_constant;
static volatile float monitor_angular_acceleration_constant;
static volatile float monitor_identified_torque_constant;
static volatile float monitor_identified_resistance;
static volatile float monitor_identified_yaw_inertia;

static volatile bool  monitor_gyroscope_scale_valid;
static volatile float monitor_gyroscope_scale;

// NOLINTEND(cppcoreguidelines-avoid-non-const-global-variables)

namespace micras {
// NOLINTBEGIN(cppcoreguidelines-avoid-non-const-global-variables) the DMA writes here
static std::array<uint8_t, bluetooth_rx_buffer_size> bluetooth_rx_buffer;
static std::array<uint8_t, bluetooth_tx_buffer_size> bluetooth_tx_buffer;

// NOLINTEND(cppcoreguidelines-avoid-non-const-global-variables)

Micras::Micras() :
    bluetooth{bluetooth_config, bluetooth_rx_buffer, bluetooth_tx_buffer},
    link{bluetooth, variables, *this, {.loop_time_us = loop_time_us}} {
    this->fsm.add_state(this->init_state);
    this->fsm.add_state(this->idle_state);
    this->fsm.add_state(this->wait_for_run_state);
    this->fsm.add_state(this->run_state);
    this->fsm.add_state(this->plan_state);
    this->fsm.add_state(this->save_state);
    this->fsm.add_state(this->wait_for_calibrate_state);
    this->fsm.add_state(this->calibrate_state);
    this->fsm.add_state(this->wait_for_identify_state);
    this->fsm.add_state(this->identify_state);
    this->fsm.add_state(this->wait_for_gyroscope_state);
    this->fsm.add_state(this->calibrate_gyroscope_state);
    this->fsm.add_state(this->error_state);

    this->register_variables();
}

void Micras::register_variables() {
    static constexpr std::array<std::string_view, 4> sensor_names{"0", "1", "2", "3"};

    for (uint8_t i = 0; i < nav::number_of_wall_sensors; i++) {
        this->variables.add("wall/", sensor_names.at(i), this->wall_sensors.get_reading(i).distance, {.stream = true});
    }

    this->variables.add("imu/", "gyro_x", this->telemetry.angular_velocity.at(0), {.stream = true});
    this->variables.add("imu/", "gyro_y", this->telemetry.angular_velocity.at(1), {.stream = true});
    this->variables.add("imu/", "gyro_z", this->telemetry.angular_velocity.at(2), {.stream = true});
    this->variables.add("imu/", "accel_x", this->telemetry.linear_acceleration.at(0), {.stream = true});
    this->variables.add("imu/", "accel_y", this->telemetry.linear_acceleration.at(1), {.stream = true});
    this->variables.add("imu/", "accel_z", this->telemetry.linear_acceleration.at(2), {.stream = true});
    this->variables.add("", "battery_voltage", this->telemetry.battery_voltage, {.stream = true});

    this->variables.add("loop/", "elapsed_time", this->elapsed_time, {.stream = true});
    this->variables.add("loop/", "worst_time_us", this->worst_loop_time_us, {.stream = true});
    this->variables.add("loop/", "missed_ticks", this->missed_ticks, {.stream = true});
    this->variables.add("loop/", "saturated_iterations", this->saturated_iterations, {.stream = true});

    const nav::State& state = this->localizer.get_state();

    this->variables.add("pose/", "x", state.pose.position.x, {.stream = true});
    this->variables.add("pose/", "y", state.pose.position.y, {.stream = true});
    this->variables.add("pose/", "orientation", state.pose.orientation, {.stream = true});
    this->variables.add("pose/", "linear_speed", state.velocity.linear, {.stream = true});
    this->variables.add("pose/", "angular_speed", state.velocity.angular, {.stream = true});

    const nav::Reference& reference = this->mission.get_reference();

    this->variables.add("reference/", "x", reference.pose.position.x, {.stream = true});
    this->variables.add("reference/", "y", reference.pose.position.y, {.stream = true});
    this->variables.add("reference/", "orientation", reference.pose.orientation, {.stream = true});
    this->variables.add("reference/", "linear_speed", reference.twist.linear, {.stream = true});
    this->variables.add("reference/", "angular_speed", reference.twist.angular, {.stream = true});

    const nav::Controller::Status& control = this->controller.get_status();

    this->variables.add("control/", "along_error", control.along_error, {.stream = true});
    this->variables.add("control/", "across_error", control.across_error, {.stream = true});
    this->variables.add("control/", "orientation_error", control.orientation_error, {.stream = true});
    this->variables.add("control/", "forward_feed_forward", control.forward_feed_forward, {.stream = true});
    this->variables.add("control/", "rotation_feed_forward", control.rotation_feed_forward, {.stream = true});
    this->variables.add("control/", "forward_feedback", control.forward_feedback, {.stream = true});
    this->variables.add("control/", "rotation_feedback", control.rotation_feedback, {.stream = true});

    this->variables.add("", "objective", this->objective, {.stream = true, .write = true, .idle = true});
    this->variables.add("", "run_profile", this->run_profile, {.stream = true, .write = true, .persist = true});
    this->variables.add("", "maze", this->mission.get_maze(), {.persist = true});

    this->link.register_variables(this->variables, "link/");

    this->maze_storage.restore(this->variables);
}

void Micras::update() {
    const uint32_t ticks = this->tick.wait();

    this->missed_ticks += ticks - 1;
    this->elapsed_time = static_cast<float>(ticks) * loop_time;
    this->watchdog.refresh();

    this->button.update();
    this->buzzer.update();
    this->interface.update();

    if (this->interface.acknowledge_event(Interface::Event::PROFILE_MOVED)) {
        this->run_profile = this->interface.get_profile();
    }

    this->battery.update();
    this->fan.update();
    this->imu.update();
    this->torque_sensors.update();
    this->wall_sensors.update();
    this->bluetooth.update();

    this->measurements = this->measure();
    this->localizer.predict(this->measurements, this->elapsed_time);

    this->fsm.update();
    this->publish();

    const uint32_t timestamp_us = this->telemetry_stopwatch.elapsed_time_us();

    this->link.poll(this->is_idle());
    this->link.pump(timestamp_us);

    this->worst_loop_time_us = std::max(this->worst_loop_time_us, this->tick.elapsed_time_us());
}

bool Micras::check_initialization() const {
    return this->battery.was_initialized() and this->fan.was_initialized() and this->locomotion.was_initialized() and
           this->torque_sensors.was_initialized() and this->argb.was_initialized() and
           this->buzzer.was_initialized() and this->imu.was_initialized() and
           this->rotary_sensor_left.was_initialized() and this->rotary_sensor_right.was_initialized() and
           this->wall_sensors.was_initialized();
}

void Micras::stop() {
    this->wall_sensors.turn_off();
    this->locomotion.stop();
    this->locomotion.disable();
    this->fan.stop();
}

bool Micras::acknowledge_event(Interface::Event event) {
    return this->interface.acknowledge_event(event);
}

void Micras::send_event(Interface::Event event) {
    this->interface.send_event(event);
}

core::Objective Micras::get_objective() const {
    return this->objective;
}

void Micras::set_objective(core::Objective objective) {
    this->objective = objective;
}

Micras::Maintenance Micras::get_maintenance() const {
    const bool diagonal = this->is_selected(Interface::Profile::DIAGONAL);
    const bool boost = this->is_selected(Interface::Profile::BOOST);
    const bool risky = this->is_selected(Interface::Profile::RISKY);

    if (diagonal and not boost and not risky) {
        return Maintenance::DRIVE;
    }

    if (boost and not diagonal and not risky) {
        return Maintenance::GYROSCOPE;
    }

    return Maintenance::WALL_SENSORS;
}

void Micras::prepare() {
    this->wall_sensors.turn_on();

    if (this->objective == core::Objective::SOLVE and this->is_selected(Interface::Profile::FAN)) {
        this->fan.enable();
        this->fan.set_speed(fan_speed);
    }
}

void Micras::rest() {
    this->localizer.correct_at_rest(this->measurements, this->elapsed_time);
}

void Micras::start_run() {
    this->crash_count = 0;
    this->locomotion.enable();

    if (this->objective != core::Objective::RETURN) {
        this->localizer.reset(this->mission.get_start_pose(), this->measurements);
    }

    this->mission.start(this->objective);
}

nav::Mission::Status Micras::run() {
    this->localizer.correct(this->measurements, this->wall_model, this->mission.get_maze());

    const nav::Mission::Status status = this->mission.update(
        this->measurements, this->localizer, this->elapsed_time, this->controller.get_time_scale()
    );

    if (status == nav::Mission::Status::RUNNING) {
        this->follow(this->mission.get_reference());
    } else {
        this->locomotion.stop();
    }

    return status;
}

bool Micras::check_crash() {
    const bool over_threshold =
        std::hypot(this->measurements.acceleration.x, this->measurements.acceleration.y) > crash_acceleration;

    this->crash_count = over_threshold ? static_cast<uint8_t>(std::min(this->crash_count + 1, 255)) : 0;

    return this->crash_count >= crash_debounce;
}

void Micras::start_plan() {
    this->mission.begin_plan(this->get_run_profile());
}

bool Micras::plan() {
    return this->mission.update_plan(plan_nodes_per_iteration);
}

bool Micras::has_route() const {
    return this->mission.has_route();
}

void Micras::save_maze() {
    const auto extension = this->watchdog.extend(flash_watchdog_timeout_ms);

    this->maze_storage.save(this->variables);
}

void Micras::start_calibration() {
    this->wall_sensors.turn_on();

    if (this->calibration_type == CalibrationType::SIDE_WALLS) {
        this->wall_sensors.calibrate_sensor(wall_sensors_index.left);
        this->wall_sensors.calibrate_sensor(wall_sensors_index.right);
    } else {
        this->wall_sensors.calibrate_sensor(wall_sensors_index.left_front);
        this->wall_sensors.calibrate_sensor(wall_sensors_index.right_front);
    }
}

bool Micras::calibrate() {
    if (this->wall_sensors.is_calibrating()) {
        return false;
    }

    this->calibration_type = this->calibration_type == CalibrationType::SIDE_WALLS ? CalibrationType::FRONT_WALL :
                                                                                     CalibrationType::SIDE_WALLS;

    return true;
}

bool Micras::is_calibration_complete() const {
    return this->calibration_type == CalibrationType::SIDE_WALLS;
}

void Micras::start_identification() {
    this->locomotion.enable();
    this->drive_identification.start(this->measurements);
}

bool Micras::identify() {
    const nav::Controller::Command command = this->drive_identification.update(this->measurements, this->elapsed_time);

    this->locomotion.set_command(command.forward, command.rotation);

    if (not this->drive_identification.is_finished()) {
        return false;
    }

    const nav::RobotModel identified = this->drive_identification.get_model();

    monitor_identification_valid = this->drive_identification.is_valid();
    monitor_breakaway_voltage = this->drive_identification.get_breakaway_voltage();
    monitor_linear_static_friction = this->drive_identification.get_linear().static_friction;
    monitor_linear_speed_constant = this->drive_identification.get_linear().speed_constant;
    monitor_linear_acceleration_constant = this->drive_identification.get_linear().acceleration_constant;
    monitor_angular_static_friction = this->drive_identification.get_angular().static_friction;
    monitor_angular_speed_constant = this->drive_identification.get_angular().speed_constant;
    monitor_angular_acceleration_constant = this->drive_identification.get_angular().acceleration_constant;
    monitor_identified_torque_constant = identified.drive.torque_constant;
    monitor_identified_resistance = identified.drive.resistance;
    monitor_identified_yaw_inertia = identified.chassis.yaw_inertia;

    return true;
}

void Micras::start_gyroscope_calibration() {
    this->locomotion.enable();
    this->gyroscope_calibration.start(this->localizer.get_pose(), this->dynamics.get_angular_limits(search_profile));
}

bool Micras::calibrate_gyroscope() {
    this->follow(
        this->gyroscope_calibration.update(this->measurements, this->localizer.get_gyroscope_bias(), this->elapsed_time)
    );

    if (not this->gyroscope_calibration.is_finished()) {
        return false;
    }

    monitor_gyroscope_scale_valid = this->gyroscope_calibration.is_valid();
    monitor_gyroscope_scale = this->gyroscope_calibration.get_scale();

    return true;
}

nav::Measurements Micras::measure() const {
    nav::Measurements sampled{
        .left_wheel_angle = this->rotary_sensor_left.get_position(),
        .right_wheel_angle = this->rotary_sensor_right.get_position(),
        .angular_rate = this->imu.get_angular_velocity(proxy::Imu::Axis::Z),
        .acceleration =
            {.x = this->imu.get_linear_acceleration(proxy::Imu::Axis::X),
             .y = this->imu.get_linear_acceleration(proxy::Imu::Axis::Y)},
        .imu_is_new = this->imu.is_new(),
        .walls = {},
    };

    for (uint8_t i = 0; i < nav::number_of_wall_sensors; i++) {
        const proxy::WallSensors::Reading& reading = this->wall_sensors.get_reading(i);

        sampled.walls.at(i) = {
            .distance = reading.distance,
            .slow_distance = reading.slow_distance,
            .valid = reading.valid,
            .is_new = reading.is_new,
        };
    }

    return sampled;
}

nav::RunProfile Micras::get_run_profile() const {
    return make_run_profile(
        this->is_selected(Interface::Profile::DIAGONAL), this->is_selected(Interface::Profile::BOOST),
        this->is_selected(Interface::Profile::RISKY), this->is_selected(Interface::Profile::FAN)
    );
}

bool Micras::is_selected(Interface::Profile option) const {
    return (this->run_profile & std::to_underlying(option)) != 0;
}

void Micras::follow(const nav::Reference& reference) {
    const nav::Controller::Command   command = this->controller.update(reference, this->localizer.get_state());
    const proxy::Locomotion::Command applied = this->locomotion.set_command(command.forward, command.rotation);

    if (applied.linear != command.forward or applied.angular != command.rotation) {
        this->saturated_iterations++;
    }
}

void Micras::publish() {
    this->telemetry.angular_velocity = {
        this->imu.get_angular_velocity(proxy::Imu::Axis::X),
        this->imu.get_angular_velocity(proxy::Imu::Axis::Y),
        this->imu.get_angular_velocity(proxy::Imu::Axis::Z),
    };

    this->telemetry.linear_acceleration = {
        this->imu.get_linear_acceleration(proxy::Imu::Axis::X),
        this->imu.get_linear_acceleration(proxy::Imu::Axis::Y),
        this->imu.get_linear_acceleration(proxy::Imu::Axis::Z),
    };

    this->telemetry.battery_voltage = this->battery.get_voltage();

    const nav::State&              state = this->localizer.get_state();
    const nav::Localizer::Status&  filter = this->localizer.get_status();
    const nav::Reference&          reference = this->mission.get_reference();
    const nav::Controller::Status& control = this->controller.get_status();

    monitor_pose_x = state.pose.position.x;
    monitor_pose_y = state.pose.position.y;
    monitor_pose_orientation = state.pose.orientation;
    monitor_linear_speed = state.velocity.linear;
    monitor_angular_speed = state.velocity.angular;
    monitor_gyroscope_bias = this->localizer.get_gyroscope_bias();
    monitor_position_deviation = this->localizer.get_position_deviation();
    monitor_orientation_deviation = this->localizer.get_orientation_deviation();
    monitor_innovation_level = filter.innovation_level;
    monitor_corrections_accepted = filter.accepted;
    monitor_corrections_rejected = filter.rejected;
    monitor_edges_used = filter.edges;

    monitor_reference_x = reference.pose.position.x;
    monitor_reference_y = reference.pose.position.y;
    monitor_reference_orientation = reference.pose.orientation;
    monitor_reference_linear_speed = reference.twist.linear;
    monitor_reference_angular_speed = reference.twist.angular;
    monitor_along_error = control.along_error;
    monitor_across_error = control.across_error;
    monitor_orientation_error = control.orientation_error;
    monitor_forward_feed_forward = control.forward_feed_forward;
    monitor_rotation_feed_forward = control.rotation_feed_forward;
    monitor_forward_feedback = control.forward_feedback;
    monitor_rotation_feedback = control.rotation_feedback;

    // NOLINTBEGIN(cppcoreguidelines-pro-bounds-constant-array-index) the arrays have one entry per sensor
    for (uint8_t i = 0; i < nav::number_of_wall_sensors; i++) {
        monitor_wall_distances[i] = this->measurements.walls.at(i).distance;
        monitor_wall_reference_readings[i] = this->wall_sensors.get_reference_reading(i);
        monitor_wall_calibration_spreads[i] = this->wall_sensors.get_calibration_spread(i);
    }
    // NOLINTEND(cppcoreguidelines-pro-bounds-constant-array-index)

    monitor_worst_loop_time_us = this->worst_loop_time_us;
    monitor_missed_ticks = this->missed_ticks;
    monitor_saturated_iterations = this->saturated_iterations;
    monitor_route_time = this->mission.get_route_time();
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

        case Command::SAVE:
            if (not this->is_idle()) {
                return comm::CommandResult::REFUSED;
            }

            this->save_maze();
            return comm::CommandResult::OK;

        case Command::RESET:
            if (not this->is_idle()) {
                return comm::CommandResult::REFUSED;
            }

            this->localizer.reset(this->mission.get_start_pose(), this->measurements);
            return comm::CommandResult::OK;
    }

    static_cast<void>(argument);
    return comm::CommandResult::UNKNOWN;
}
}  // namespace micras
