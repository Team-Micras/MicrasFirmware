/**
 * @file
 */

#ifndef MICRAS_HPP
#define MICRAS_HPP

#include <array>
#include <cstdint>
#include <optional>
#include <utility>

#include "constants.hpp"
#include "micras/comm/link.hpp"
#include "micras/command.hpp"
#include "micras/core/fsm.hpp"
#include "micras/core/types.hpp"
#include "micras/core/variable_pool.hpp"
#include "micras/interface.hpp"
#include "micras/nav/controller.hpp"
#include "micras/nav/drive_identification.hpp"
#include "micras/nav/gyroscope_calibration.hpp"
#include "micras/nav/localizer.hpp"
#include "micras/nav/measurements.hpp"
#include "micras/nav/motion_limits.hpp"
#include "micras/nav/speed_ramp.hpp"
#include "micras/nav/wall_model.hpp"
#include "micras/proxy/microsecond_clock.hpp"
#include "micras/states/brake.hpp"
#include "micras/states/calibrate.hpp"
#include "micras/states/calibrate_gyroscope.hpp"
#include "micras/states/error.hpp"
#include "micras/states/identify.hpp"
#include "micras/states/idle.hpp"
#include "micras/states/init.hpp"
#include "micras/states/plan.hpp"
#include "micras/states/run.hpp"
#include "micras/states/save.hpp"
#include "micras/states/wait.hpp"
#include "target.hpp"

namespace micras {
/**
 * @brief Class for controlling the Micras robot.
 *
 * @details This is the only place where the proxies and the navigation meet. Once per iteration the
 * sensors are sampled into plain measurements, the navigation turns them into a command and the
 * command is applied to the motors, under a state machine that decides what the robot is doing.
 *
 * @note An object of this class holds the arrays of the route planner, so it is far too large for
 * the stack and belongs in static storage.
 */
class Micras : public comm::ICommandHandler {
public:
    /**
     * @brief Procedures that an extra long press of the button can start, chosen by the switches.
     *
     * @note With the racing line, boost and risky switches off it is the calibration of the wall
     * sensors. The racing line switch alone selects the identification of the drive train and the
     * boost switch alone the calibration of the gyroscope scale.
     */
    enum class Maintenance : uint8_t {
        WALL_SENSORS = 0,
        DRIVE = 1,
        GYROSCOPE = 2,
    };

    /**
     * @brief Commands the link can ask the robot to run, and why they are refused.
     */
    ///@{
    using Command = micras::Command;
    using Reason = micras::Reason;
    ///@}

    /**
     * @brief What made the robot stop, as the fault variable of the pool shows it.
     */
    enum class Fault : uint8_t {
        NONE = 0,
        CRASH = 1,
        SATURATION = 2,
        IMU = 3,
        INITIALIZATION = 4,
    };

    /**
     * @brief Construct a new Micras object.
     */
    Micras();

    /**
     * @brief Update the controller loop of the robot.
     */
    void update();

    /**
     * @brief Check if the robot was correctly initialized.
     *
     * @note Every proxy the robot drives with has to have started. A start that followed a reset by
     * the watchdog also fails, so that an error that keeps resetting the microcontroller stops in
     * the error state where it can be seen, instead of looping through boots, and so does a core
     * clocked above what its option bytes allow.
     *
     * @return True if every device was initialized, false otherwise.
     */
    bool check_initialization() const;

    /**
     * @brief Keep in the pool that the start failed, as the initialization fault.
     *
     * @note No command clears it: leaving the error state it led to is refused, since a robot that
     * failed its start cannot be trusted to move until it is reset.
     */
    void record_initialization_fault();

    /**
     * @brief Put the estimate of the pose back at the start of the maze.
     *
     * @note The initialization does it once it has finished, so the pose is where the robot is
     * placed before the first run, and not at the corner of the maze.
     */
    void place_at_start();

    /**
     * @brief Stop the robot, turning its sensors and actuators off and dropping whatever procedure
     * was under way.
     *
     * @note A calibration of the wall sensors in progress is abandoned, since it would otherwise go
     * on averaging readings with the emitters off, and the next one starts over with the side
     * sensors.
     */
    void stop();

    /**
     * @brief Get the value of an event and reset it.
     *
     * @param event The event to get.
     * @return True if the event happened, false otherwise.
     */
    bool acknowledge_event(Interface::Event event);

    /**
     * @brief Forget the presses of the button that no state has acted on yet.
     */
    void discard_presses();

    /**
     * @brief Send an event to the interface.
     *
     * @param event The event to send.
     */
    void send_event(Interface::Event event);

    /**
     * @brief Check if a stop arrived while the maze was being saved, and forget it.
     *
     * @return True if the robot has to stop once the save ends.
     */
    bool acknowledge_deferred_stop();

    /**
     * @brief Get the current objective of the robot.
     *
     * @return The current objective of the robot.
     */
    core::Objective get_objective() const;

    /**
     * @brief Set the current objective of the robot.
     *
     * @param objective The new objective of the robot.
     */
    void set_objective(core::Objective objective);

    /**
     * @brief Get the procedure the switches select for an extra long press of the button.
     *
     * @return The procedure.
     */
    Maintenance get_maintenance() const;

    /**
     * @brief Get the robot ready to move: sensors on, and the fan too if what follows uses it.
     *
     * @note A fast run and a maintenance procedure run the fan when its switch is on, a search never.
     * The current sensors are zeroed here, while the motors are still disabled.
     *
     * @param run Whether a run follows, rather than a maintenance procedure.
     */
    void prepare(bool run);

    /**
     * @brief Check if what prepare started is ready.
     *
     * @note The fan ramps its speed up at a limited rate, and the run is planned with the traction
     * of the fan at full speed, so a run must not start before the fan gets there.
     *
     * @return True once the fan runs at the speed it was asked for.
     */
    bool is_prepared() const;

    /**
     * @brief Keep estimating the bias of the gyroscope while the robot waits to move.
     */
    void rest();

    /**
     * @brief Start the run of the current objective.
     */
    void start_run();

    /**
     * @brief Advance the run of the current objective by one iteration.
     *
     * @return The progress of the run.
     */
    nav::Mission::Status run();

    /**
     * @brief Check if something went wrong that the robot has to stop for.
     *
     * @note Three things are checked: an acceleration over the crash threshold, the motors
     * saturated for longer than the saturation timeout, which is what a robot held against a wall
     * or with an encoder reversed looks like, and the inertial measurement unit silent for longer
     * than its timeout, which is what it looks like after a brownout, since it comes back powered
     * down. The one that fired is kept in the pool until the next run starts, since the state alone
     * does not tell them apart.
     *
     * @return True if the robot has to stop.
     */
    bool check_fault();

    /**
     * @brief Load the maze from the non-volatile storage and start planning a fast run.
     */
    void start_plan();

    /**
     * @brief Advance the planning of the fast run.
     *
     * @return True if the planning has finished.
     */
    bool plan();

    /**
     * @brief Check if the planning found a route.
     *
     * @return True if there is a route to run.
     */
    bool has_route() const;

    /**
     * @brief Save the maze to the non-volatile storage.
     *
     * @note This stalls the core for seconds, so it is only to be called with the robot stopped.
     * The LED is on for as long as it lasts, since switching the robot off then loses the saved
     * map, and the loop is paced again from its end. A failed save is counted in the telemetry and
     * nothing else changes, since the map is still in memory for the runs until the next reset.
     *
     * @return True if the maze was saved.
     */
    bool save_maze();

    /**
     * @brief Start bringing the robot to a standstill, from the state the stop arrived in.
     *
     * @note A run brakes along its path, the calibration of the gyroscope ramps its rotation down
     * and holds the angle the ramp ends at until the robot is at rest, and the identification of the
     * drive train holds a null command with the drivers on, so the bridge shorts the motors and they
     * brake with their own back EMF. A second stop during the brake replaces whatever brake it was
     * with the brake of the wheels, which start_wheel_brake() starts.
     */
    void start_brake();

    /**
     * @brief Advance the brake by one iteration.
     *
     * @note Every brake but that of a run ends once the robot has been at rest for the settle time
     * of the executor, since the speed only settles gradually and the estimate passes the speeds
     * of rest a little before the body does.
     *
     * @return True once the robot stands still.
     */
    bool brake();

    /**
     * @brief Replace the brake under way by a brake of the wheels alone, after a second stop.
     *
     * @note The speeds the wheels and the gyroscope measure are ramped down to zero as hard as the
     * stop profile lets the tires brake, with the downforce of the fan if it is at speed, through
     * the loop on the speeds alone, with no path and no pose. Once the ramp is at rest the motors
     * are shorted until the robot has settled.
     */
    void start_wheel_brake();

    /**
     * @brief Count the time the robot has been at rest, over this iteration.
     *
     * @return True once it has been at rest for the settle time of the executor.
     */
    bool has_settled();

    /**
     * @brief Start the calibration of the pair of wall sensors that is next in line.
     */
    void start_calibration();

    /**
     * @brief Check on the calibration of the wall sensors.
     *
     * @return True once the pair of sensors being calibrated is done.
     */
    bool calibrate();

    /**
     * @brief Check if both pairs of wall sensors were calibrated.
     *
     * @return True if the next calibration starts over with the first pair.
     */
    bool is_calibration_complete() const;

    /**
     * @brief Start the identification of the drive train.
     */
    void start_identification();

    /**
     * @brief Advance the identification of the drive train by one iteration.
     *
     * @return True if the identification has finished.
     */
    bool identify();

    /**
     * @brief Start the calibration of the gyroscope scale.
     */
    void start_gyroscope_calibration();

    /**
     * @brief Advance the calibration of the gyroscope scale by one iteration.
     *
     * @return True if the calibration has finished.
     */
    bool calibrate_gyroscope();

    /**
     * @brief Run a command that arrived over the link, if the current state accepts it.
     *
     * @param code Command to run, one of Command.
     * @param argument Argument of the command, which no command uses.
     * @return Whether the command ran, and why not otherwise, one of Reason.
     */
    comm::CommandReply handle_command(uint8_t code, uint32_t argument) override;

    /**
     * @brief Check if the robot is stopped.
     *
     * @note This is what gates the guarded writes and the commands that block for seconds, so it
     * is the state of the machine rather than a flag anything else maintains.
     *
     * @return True if the robot is idle, false otherwise.
     */
    bool is_idle() const;

    /**
     * @brief Get the robot constructed last.
     *
     * @note For tools that run the firmware on a host, such as the simulator, which reads every
     * registered variable after each step of its world. The robot lives in a static of main, which
     * nothing else can name. Nothing on the robot calls this: it costs one pointer stored at
     * construction.
     *
     * @return The robot, or null before one is constructed.
     */
    static const Micras* get_instance();

    /**
     * @brief Get the pool of every variable the robot registers.
     *
     * @return The pool.
     */
    const core::VariablePool& get_variables() const;

    /**
     * @brief Get the state the robot's state machine is in.
     *
     * @return Id of the state, one of State.
     */
    uint8_t get_state() const;

private:
    /**
     * @brief Values in the variable pool that no object holds at a stable address.
     *
     * @note Some of what is worth watching is computed on the way out of its owner: the battery is
     * scaled into volts, the deviations of the estimate come out of its covariance. Publishing means
     * copying those into somewhere that stays put. The route time is zero while no route is planned.
     * The restarts of the converters are counted by the HAL, in a static member. The failed saves are
     * counted as they fail. The results of the maintenance procedures are copied once, when a
     * procedure ends. The revision of the maze lets an application read the map again only when it
     * changed.
     */
    struct Telemetry {
        std::array<float, 3>                           angular_velocity{};
        std::array<float, 3>                           linear_acceleration{};
        float                                          battery_voltage{};
        uint32_t                                       adc_restarts{};
        uint32_t                                       failed_saves{};
        float                                          gyroscope_bias{};
        float                                          position_deviation{};
        float                                          orientation_deviation{};
        float                                          route_time{};
        std::array<float, nav::number_of_wall_sensors> wall_reference_readings{};
        std::array<float, nav::number_of_wall_sensors> wall_calibration_spreads{};
        bool                                           identification_valid{};
        float                                          breakaway_voltage{};
        nav::DriveIdentification::Axis                 linear_drive{};
        nav::DriveIdentification::Axis                 angular_drive{};
        float                                          torque_constant{};
        float                                          resistance{};
        float                                          yaw_inertia{};
        bool                                           gyroscope_scale_valid{};
        float                                          gyroscope_scale{};
        uint32_t                                       maze_revision{};
    };

    /**
     * @brief Register every variable the robot exposes, and load the ones the flash memory holds.
     *
     * @note Each owner registers its own members, so the values are sampled where they already
     * live and nothing has to be copied once per iteration to keep a second set up to date.
     */
    void register_variables();

    /**
     * @brief Enum for the type of calibration being performed.
     */
    enum class CalibrationType : uint8_t {
        SIDE_WALLS = 0,  // Calibrate the sensors that look at the side walls, in a corridor with no wall ahead.
        FRONT_WALL = 1,  // Calibrate the sensors that look forward, facing a wall.
    };

    /**
     * @brief Sample every sensor the navigation needs.
     *
     * @note The inertial measurement unit sits with its pin 1 at the front left of the board, so
     * its Y axis points forward and its X axis to the right. Its Z axis points up, which makes the
     * yaw rate its Z reading as it is.
     *
     * @return The measurements of this iteration.
     */
    nav::Measurements measure() const;

    /**
     * @brief Get the profile of a fast run, from the options of the run profile.
     *
     * @return The profile.
     */
    nav::RunProfile get_run_profile() const;

    /**
     * @brief Check if an option of the run profile is selected.
     *
     * @param option The option.
     * @return True if the option is selected.
     */
    bool is_selected(Interface::Profile option) const;

    /**
     * @brief Make the robot follow a reference.
     *
     * @param reference What the robot should be doing at this instant.
     */
    void follow(const nav::Reference& reference);

    /**
     * @brief Apply a command of the controller, and count the iterations it saturates the motors.
     *
     * @param command The command.
     */
    void drive(const nav::Controller::Command& command);

    /**
     * @brief Copy what is worth watching and has no stable address to the telemetry.
     */
    void publish();

    /**
     * @brief Forget the counts of the faults and the last fault, before the robot starts to move.
     */
    void clear_faults();

    /**
     * @brief Run a command the current state accepts.
     *
     * @param command The command.
     * @return Whether it ran, and why not otherwise.
     */
    comm::CommandReply carry_out(Command command);

    /**
     * @brief Stop whatever the robot is doing and make it idle.
     *
     * @note A robot driving its motors brakes to a standstill first, in the brake state. A second
     * stop once the brake is under way trusts neither the path nor the pose: it drops the brake and
     * ramps the speeds the wheels and the gyroscope measure down to zero as hard as the tires
     * allow, through the loop on the speeds alone, then holds a null command with the drivers on,
     * so the bridge shorts the motors, until the robot has settled. The faults are still watched,
     * the timeout of the first brake still bounds it, and the robot is then idle with the drivers
     * off and the presses of the button forgotten. A stop that arrives before the brake has started,
     * or once the wheels are already braking, only confirms it: it neither restarts the ramp nor
     * clears the time the robot has been at rest. The robot stays in the error state, and in the initialization it has
     * not finished, if it is there. During a save the stop waits for the save to end.
     *
     * @return Whether the robot stopped, or will once the maze is saved.
     */
    comm::CommandReply halt();

    /**
     * @brief Check if the robot is at rest, from the estimate of its speeds.
     *
     * @return True if both speeds are below those the robot is taken to have settled at.
     */
    bool is_at_rest() const;

    /**
     * @brief Make the robot idle again after an error, unless the error came from the start.
     *
     * @return Whether the robot left the error state, and why not otherwise.
     */
    comm::CommandReply leave_error();

    /**
     * @brief Publish the state of the state machine, and report it over the link when it changed.
     *
     * @note Called after the link acted on its message, which may have changed the state, so the
     * published state is always the one the next iteration runs.
     *
     * @param timestamp_us Time of the iteration, on the clock the samples are stamped with.
     */
    void report_state(uint32_t timestamp_us);

    /**
     * @brief Watchdog, started before anything that could hang.
     */
    proxy::Watchdog watchdog{watchdog_config};

    /**
     * @brief Longer timeout of the watchdog while the robot is being constructed.
     *
     * @note Some proxies wait for their chips as they start, the inertial measurement unit for
     * 40 ms, which is longer than the timeout of the control loop. The constructor ends it once
     * every member is built.
     */
    std::optional<proxy::Watchdog::Extension> startup_extension{std::in_place, watchdog, stopped_watchdog_timeout_ms};

    /**
     * @brief Longer timeout of the watchdog while a fast run is planned.
     *
     * @note The robot is stopped then, and the iteration that chooses among the candidate routes
     * and starts the racing line does all of that at once, which can outlast the timeout of the
     * control loop. It lives from the start of the planning to its end.
     */
    std::optional<proxy::Watchdog::Extension> plan_extension;

    /**
     * @brief Pace of the control loop.
     */
    proxy::Tick tick{tick_config};

    /**
     * @brief Sensors and actuators.
     *
     * @note Every proxy is owned here, by value, for the whole lifetime of the program, and the
     * objects that use them borrow them by reference.
     */
    ///@{
    proxy::Battery       battery{battery_config};
    proxy::Fan           fan{fan_config};
    proxy::Locomotion    locomotion{locomotion_config};
    proxy::Storage       maze_storage{maze_storage_config};
    proxy::TorqueSensors torque_sensors{torque_sensors_config};
    ///@}

    /**
     * @brief Interface proxies with the external world.
     */
    ///@{
    proxy::Argb            argb{argb_config};
    proxy::BluetoothSerial bluetooth;
    proxy::Button          button{button_config};
    proxy::Buzzer          buzzer{buzzer_config};
    proxy::DipSwitch       dip_switch{dip_switch_config};
    proxy::Led             led{led_config};
    ///@}

    /**
     * @brief Sensors sampled into the measurements of the navigation.
     */
    ///@{
    proxy::Imu          imu{imu_config};
    proxy::RotarySensor rotary_sensor_left{rotary_sensor_left_config};
    proxy::RotarySensor rotary_sensor_right{rotary_sensor_right_config};
    proxy::WallSensors  wall_sensors{wall_sensors_config};
    ///@}

    /**
     * @brief High level objects.
     */
    ///@{
    nav::Dynamics             dynamics{dynamics_config};
    nav::WallModel            wall_model{wall_model_config};
    nav::Localizer            localizer{localizer_config};
    nav::Controller           controller{controller_config};
    nav::Mission              mission{dynamics, wall_model, mission_config};
    nav::DriveIdentification  drive_identification{drive_identification_config};
    nav::GyroscopeCalibration gyroscope_calibration{gyroscope_calibration_config};
    nav::SpeedRamp            speed_ramp;
    ///@}

    /**
     * @brief Class for controlling the interface with the external world.
     */
    Interface interface{button, dip_switch, led};

    /**
     * @brief States of the robot, held by value and borrowed by the state machine.
     */
    ///@{
    InitState               init_state{State::INIT, *this};
    IdleState               idle_state{State::IDLE, *this};
    WaitState               wait_for_run_state{State::WAIT_FOR_RUN, *this, State::RUN};
    RunState                run_state{State::RUN, *this};
    PlanState               plan_state{State::PLAN, *this};
    SaveState               save_state{State::SAVE, *this};
    WaitState               wait_for_calibrate_state{State::WAIT_FOR_CALIBRATE, *this, State::CALIBRATE};
    CalibrateState          calibrate_state{State::CALIBRATE, *this};
    WaitState               wait_for_identify_state{State::WAIT_FOR_IDENTIFY, *this, State::IDENTIFY};
    IdentifyState           identify_state{State::IDENTIFY, *this};
    WaitState               wait_for_gyroscope_state{State::WAIT_FOR_GYROSCOPE, *this, State::CALIBRATE_GYROSCOPE};
    CalibrateGyroscopeState calibrate_gyroscope_state{State::CALIBRATE_GYROSCOPE, *this};
    ErrorState              error_state{State::ERROR, *this};
    BrakeState              brake_state{State::BRAKE, *this, brake_timeout_ms};
    ///@}

    /**
     * @brief Sensor values as they are published, which is not always as they are stored.
     */
    Telemetry telemetry;

    /**
     * @brief Every variable exposed to the storage and to the communication link.
     */
    core::TVariablePool<max_variables> variables;

    /**
     * @brief Session the variables are watched and steered through.
     */
    comm::Link link;

    /**
     * @brief Free running clock the samples are stamped with.
     *
     * @note It is read in every iteration and right after a save, the longest stall of the loop, so
     * a wrap of the cycle counter is only lost to a stall longer than one, about 7.81 s.
     */
    proxy::MicrosecondClock telemetry_clock;

    /**
     * @brief Finite state machine for the robot.
     */
    core::TFsm<std::to_underlying(State::NUMBER_OF_STATES)> fsm{std::to_underlying(State::INIT)};

    /**
     * @brief Measurements of the current iteration.
     */
    nav::Measurements measurements{};

    /**
     * @brief Id of the state the next iteration runs, as the pool publishes it.
     */
    uint8_t state_id{std::to_underlying(State::INIT)};

    /**
     * @brief Whether a stop arrived while the maze was to be saved.
     */
    bool stop_deferred{};

    /**
     * @brief State the robot was in when a stop made it brake, which says how to brake.
     *
     * @note It is the brake state itself once a second stop has made it brake the wheels.
     */
    State braked_state{State::RUN};

    /**
     * @brief Time the robot has been at rest while it brakes, in seconds.
     */
    float settled_time{};

    /**
     * @brief Speed of the fan, as a share of the speed it runs at.
     */
    float fan_share{};

    /**
     * @brief Current objective of the robot.
     */
    core::Objective objective{core::Objective::EXPLORE};

    /**
     * @brief Options of the next run, written by the switches and by the link alike.
     *
     * @note Last writer wins, which is a rule that can be predicted from the outside. The switches
     * write it when they move, the link whenever it likes. It is not saved: at boot it is what the
     * switches say, since a saved value would win exactly when every switch is off, which is when
     * nobody expects it.
     */
    uint8_t run_profile{};

    /**
     * @brief Current type of calibration being performed.
     */
    CalibrationType calibration_type{CalibrationType::SIDE_WALLS};

    /**
     * @brief Number of consecutive iterations with an acceleration over the crash threshold.
     */
    uint8_t crash_count{};

    /**
     * @brief Number of consecutive iterations in which the motors could not deliver the command.
     */
    uint16_t saturated_streak{};

    /**
     * @brief Number of consecutive iterations without a new sample of the inertial measurement unit.
     */
    uint16_t imu_silence{};

    /**
     * @brief Fault that last stopped the robot, or none since the last run started.
     */
    Fault fault{Fault::NONE};

    /**
     * @brief Longest control loop body observed since the last reset, in microseconds.
     *
     * @note Not acted on, but the only way to know how much of the loop budget is actually used,
     * which every performance decision depends on.
     */
    uint32_t worst_loop_time_us{};

    /**
     * @brief Time since the previous iteration, in seconds.
     *
     * @note A whole number of periods, which is one unless the previous iteration was late. Every
     * integration in the navigation takes it, so that a late iteration costs the resolution of one
     * and not the angle the robot turned during it.
     */
    float elapsed_time{loop_time};

    /**
     * @brief Number of periods of the control loop that went by without an iteration.
     *
     * @note The planning of a fast run can take more than a period per iteration and is counted
     * here too, with the robot stopped, and so are the one or two that flooding the maze takes
     * whenever a search decides a wall. The saving of the maze is not: the loop is paced again
     * from its end. What matters is that it does not grow during a fast run.
     */
    uint32_t missed_ticks{};

    /**
     * @brief Number of iterations in which the motors could not deliver the command.
     */
    uint32_t saturated_iterations{};
};
}  // namespace micras

#endif  // MICRAS_HPP
