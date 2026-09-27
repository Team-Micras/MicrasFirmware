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
#include "micras/core/variable_pool.hpp"
#include "micras/hal/adc_dma.hpp"
#include "micras/hal/mcu.hpp"
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

namespace micras {
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables) set once, by the constructor
static const Micras* last_constructed{};

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

    this->startup_extension.reset();
    this->tick.restart();

    last_constructed = this;
}

void Micras::register_variables() {
    static constexpr std::array<std::string_view, 4> sensor_names{"0", "1", "2", "3"};

    for (uint8_t i = 0; i < nav::number_of_wall_sensors; i++) {
        this->variables.add("wall/", sensor_names.at(i), this->wall_sensors.get_reading(i).distance, {.stream = true});
    }

    for (uint8_t i = 0; i < nav::number_of_wall_sensors; i++) {
        this->variables.add("wall_dark/", sensor_names.at(i), this->wall_sensors.get_reading(i).dark, {.stream = true});
    }

    this->variables.add("imu/", "gyro_x", this->telemetry.angular_velocity.at(0), {.stream = true});
    this->variables.add("imu/", "gyro_y", this->telemetry.angular_velocity.at(1), {.stream = true});
    this->variables.add("imu/", "gyro_z", this->telemetry.angular_velocity.at(2), {.stream = true});
    this->variables.add("imu/", "accel_x", this->telemetry.linear_acceleration.at(0), {.stream = true});
    this->variables.add("imu/", "accel_y", this->telemetry.linear_acceleration.at(1), {.stream = true});
    this->variables.add("imu/", "accel_z", this->telemetry.linear_acceleration.at(2), {.stream = true});
    this->variables.add("", "battery_voltage", this->telemetry.battery_voltage, {.stream = true});
    this->variables.add("", "adc_restarts", this->telemetry.adc_restarts, {});
    this->variables.add("", "failed_saves", this->telemetry.failed_saves, {});
    this->variables.add("", "fault", this->fault, {});

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

    const nav::Localizer::Status& filter = this->localizer.get_status();

    this->variables.add("localizer/", "gyroscope_bias", this->telemetry.gyroscope_bias, {.stream = true});
    this->variables.add("localizer/", "position_deviation", this->telemetry.position_deviation, {.stream = true});
    this->variables.add("localizer/", "orientation_deviation", this->telemetry.orientation_deviation, {.stream = true});
    this->variables.add("localizer/", "innovation_level", filter.innovation_level, {.stream = true});
    this->variables.add("localizer/", "accepted", filter.accepted, {.stream = true});
    this->variables.add("localizer/", "rejected", filter.rejected, {.stream = true});
    this->variables.add("localizer/", "edges", filter.edges, {.stream = true});
    this->variables.add("localizer/", "recoveries", filter.recoveries, {.stream = true});

    for (uint8_t i = 0; i < nav::number_of_wall_sensors; i++) {
        this->variables.add("wall_reference/", sensor_names.at(i), this->telemetry.wall_reference_readings.at(i), {});
        this->variables.add("wall_spread/", sensor_names.at(i), this->telemetry.wall_calibration_spreads.at(i), {});
    }

    this->variables.add("", "route_time", this->telemetry.route_time, {.stream = true});

    this->variables.add("identification/", "valid", this->telemetry.identification_valid, {});
    this->variables.add("identification/", "breakaway_voltage", this->telemetry.breakaway_voltage, {});
    this->variables.add("identification/", "linear_static_friction", this->telemetry.linear_drive.static_friction, {});
    this->variables.add("identification/", "linear_speed_constant", this->telemetry.linear_drive.speed_constant, {});
    this->variables.add(
        "identification/", "linear_acceleration_constant", this->telemetry.linear_drive.acceleration_constant, {}
    );
    this->variables.add(
        "identification/", "angular_static_friction", this->telemetry.angular_drive.static_friction, {}
    );
    this->variables.add("identification/", "angular_speed_constant", this->telemetry.angular_drive.speed_constant, {});
    this->variables.add(
        "identification/", "angular_acceleration_constant", this->telemetry.angular_drive.acceleration_constant, {}
    );
    this->variables.add("identification/", "torque_constant", this->telemetry.torque_constant, {});
    this->variables.add("identification/", "resistance", this->telemetry.resistance, {});
    this->variables.add("identification/", "yaw_inertia", this->telemetry.yaw_inertia, {});

    this->variables.add("gyroscope/", "scale_valid", this->telemetry.gyroscope_scale_valid, {});
    this->variables.add("gyroscope/", "scale", this->telemetry.gyroscope_scale, {});

    this->variables.add("", "objective", this->objective, {.stream = true, .write = true, .idle = true});
    this->variables.add("", "run_profile", this->run_profile, {.stream = true, .write = true});
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
    const float fan_share = this->fan.update() / fan_speed;

    this->localizer.set_downforce(fan_share * fan_share);
    this->imu.update();
    this->torque_sensors.update();
    this->wall_sensors.update();
    this->bluetooth.update();

    this->measurements = this->measure();
    this->imu_silence =
        this->measurements.imu_is_new ? 0 : static_cast<uint16_t>(std::min(this->imu_silence + 1, 65535));
    this->localizer.predict(this->measurements, this->elapsed_time);

    this->fsm.update();
    this->publish();

    const uint32_t timestamp_us = this->telemetry_stopwatch.elapsed_time_us();

    this->link.poll(this->is_idle());
    this->link.pump(timestamp_us);

    this->worst_loop_time_us = std::max(this->worst_loop_time_us, this->tick.elapsed_time_us());
}

bool Micras::check_initialization() const {
    return not hal::Mcu::was_reset_by_watchdog() and hal::Mcu::is_cpu_frequency_supported() and
           this->battery.was_initialized() and this->fan.was_initialized() and this->locomotion.was_initialized() and
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
    const bool racing_line = this->is_selected(Interface::Profile::RACING_LINE);
    const bool boost = this->is_selected(Interface::Profile::BOOST);
    const bool risky = this->is_selected(Interface::Profile::RISKY);

    if (racing_line and not boost and not risky) {
        return Maintenance::DRIVE;
    }

    if (boost and not racing_line and not risky) {
        return Maintenance::GYROSCOPE;
    }

    return Maintenance::WALL_SENSORS;
}

void Micras::prepare(bool run) {
    this->torque_sensors.calibrate();
    this->wall_sensors.turn_on();

    if ((not run or this->objective == core::Objective::SOLVE) and this->is_selected(Interface::Profile::FAN)) {
        this->fan.enable();
        this->fan.set_speed(fan_speed);
    }
}

bool Micras::is_prepared() const {
    return this->fan.is_at_speed();
}

void Micras::rest() {
    this->localizer.correct_at_rest(this->measurements, this->elapsed_time);
}

void Micras::start_run() {
    this->clear_faults();
    this->controller.reset();
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

bool Micras::check_fault() {
    const bool over_threshold =
        std::hypot(this->measurements.acceleration.x, this->measurements.acceleration.y) > crash_acceleration;

    this->crash_count = over_threshold ? static_cast<uint8_t>(std::min(this->crash_count + 1, 255)) : 0;

    if (this->crash_count >= crash_debounce) {
        this->fault = Fault::CRASH;
    } else if (this->saturated_streak >= saturation_timeout) {
        this->fault = Fault::SATURATION;
    } else if (this->imu_silence >= imu_timeout) {
        this->fault = Fault::IMU;
    }

    return this->fault != Fault::NONE;
}

void Micras::start_plan() {
    this->plan_extension.emplace(this->watchdog, stopped_watchdog_timeout_ms);
    this->mission.begin_plan(this->get_run_profile());
}

bool Micras::plan() {
    if (not this->mission.update_plan(plan_edges_per_iteration)) {
        return false;
    }

    this->plan_extension.reset();
    return true;
}

bool Micras::has_route() const {
    return this->mission.has_route();
}

bool Micras::save_maze() {
    const auto extension = this->watchdog.extend(stopped_watchdog_timeout_ms);

    this->led.turn_on();
    const bool saved = this->maze_storage.save(this->variables);
    this->led.turn_off();
    this->tick.restart();

    if (not saved) {
        this->telemetry.failed_saves++;
    }

    return saved;
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
    this->clear_faults();
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

    this->telemetry.identification_valid = this->drive_identification.is_valid();
    this->telemetry.breakaway_voltage = this->drive_identification.get_breakaway_voltage();
    this->telemetry.linear_drive = this->drive_identification.get_linear();
    this->telemetry.angular_drive = this->drive_identification.get_angular();
    this->telemetry.torque_constant = identified.drive.torque_constant;
    this->telemetry.resistance = identified.drive.resistance;
    this->telemetry.yaw_inertia = identified.chassis.yaw_inertia;

    return true;
}

void Micras::start_gyroscope_calibration() {
    this->clear_faults();
    this->controller.reset();
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

    this->telemetry.gyroscope_scale_valid = this->gyroscope_calibration.is_valid();
    this->telemetry.gyroscope_scale = this->gyroscope_calibration.get_scale();

    return true;
}

nav::Measurements Micras::measure() const {
    nav::Measurements sampled{
        .left_wheel_angle = this->rotary_sensor_left.get_position(),
        .right_wheel_angle = this->rotary_sensor_right.get_position(),
        .angular_rate = this->imu.get_angular_velocity(proxy::Imu::Axis::Z),
        .acceleration =
            {.x = this->imu.get_linear_acceleration(proxy::Imu::Axis::Y),
             .y = -this->imu.get_linear_acceleration(proxy::Imu::Axis::X)},
        .imu_is_new = this->imu.is_new(),
        .walls = {},
    };

    for (uint8_t i = 0; i < nav::number_of_wall_sensors; i++) {
        const proxy::WallSensors::Reading& reading = this->wall_sensors.get_reading(i);

        sampled.walls.at(i) = {
            .distance = reading.distance,
            .valid = reading.valid,
            .blind = reading.blind,
            .is_new = reading.is_new,
        };
    }

    return sampled;
}

nav::RunProfile Micras::get_run_profile() const {
    return make_run_profile(
        this->is_selected(Interface::Profile::RACING_LINE), this->is_selected(Interface::Profile::BOOST),
        this->is_selected(Interface::Profile::RISKY), this->is_selected(Interface::Profile::FAN)
    );
}

bool Micras::is_selected(Interface::Profile option) const {
    return (this->run_profile & std::to_underlying(option)) != 0;
}

void Micras::follow(const nav::Reference& reference) {
    const nav::Controller::Command command =
        this->controller.update(reference, this->localizer.get_state(), this->elapsed_time);
    const proxy::Locomotion::Command applied = this->locomotion.set_command(command.forward, command.rotation);

    if (applied.linear != command.forward or applied.angular != command.rotation) {
        this->saturated_iterations++;
        this->saturated_streak = static_cast<uint16_t>(std::min(this->saturated_streak + 1, 65535));
    } else {
        this->saturated_streak = 0;
    }
}

void Micras::clear_faults() {
    this->crash_count = 0;
    this->saturated_streak = 0;
    this->fault = Fault::NONE;
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
    this->telemetry.adc_restarts = hal::AdcDma::get_restarts();

    this->telemetry.gyroscope_bias = this->localizer.get_gyroscope_bias();
    this->telemetry.position_deviation = this->localizer.get_position_deviation();
    this->telemetry.orientation_deviation = this->localizer.get_orientation_deviation();
    this->telemetry.route_time = std::isfinite(this->mission.get_route_time()) ? this->mission.get_route_time() : 0.0F;

    for (uint8_t i = 0; i < nav::number_of_wall_sensors; i++) {
        this->telemetry.wall_reference_readings.at(i) = this->wall_sensors.get_reference_reading(i);
        this->telemetry.wall_calibration_spreads.at(i) = this->wall_sensors.get_calibration_spread(i);
    }
}

bool Micras::is_idle() const {
    return this->fsm.get_current_state_id() == std::to_underlying(State::IDLE);
}

const Micras* Micras::get_instance() {
    return last_constructed;
}

const core::VariablePool& Micras::get_variables() const {
    return this->variables;
}

uint8_t Micras::get_state() const {
    return this->fsm.get_current_state_id();
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

            return this->save_maze() ? comm::CommandResult::OK : comm::CommandResult::REFUSED;

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
