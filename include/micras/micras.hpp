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
#include "micras/calibration_record.hpp"
#include "micras/comm/link.hpp"
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
#include "micras/nav/wall_model.hpp"
#include "micras/states/calibrate.hpp"
#include "micras/states/calibrate_gyroscope.hpp"
#include "micras/states/calibrate_offsets.hpp"
#include "micras/states/check_crosstalk.hpp"
#include "micras/states/check_polarity.hpp"
#include "micras/states/check_sensors.hpp"
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
     * @brief Procedures that an extra long press of the button or the MAINTAIN command can start.
     *
     * @note For the button they are chosen by the switches: with the racing line, boost and risky
     * switches off it is the calibration of the wall sensors, the racing line switch alone selects
     * the identification of the drive train and the boost switch alone the calibration of the
     * gyroscope scale. The checks of the sensors, of the polarity and of the crosstalk, and the
     * calibration of the offsets of the wall sensors, are only reachable from the link, whose
     * command names the procedure.
     */
    enum class Maintenance : uint8_t {
        WALL_SENSORS = 0,
        DRIVE = 1,
        GYROSCOPE = 2,
        SENSORS = 3,
        POLARITY = 4,
        CROSSTALK = 5,
        WALL_OFFSETS = 6,
        NUMBER_OF_PROCEDURES = 7,
    };

    /**
     * @brief Commands the link can ask the robot to run.
     *
     * @note These are edges, not levels: each one happens once, when it arrives. Everything that
     * is a level, like the run profile, is a writable variable instead.
     *
     * @note STOP is accepted in every state. A robot that is moving stops with the STOPPED fault in
     * the error state, so that what stopped it stays visible, and one that waits to move goes back
     * to idle. RESUME leaves the error state for idle, unless the initialization failed. MAINTAIN
     * starts the procedure its argument names, one of Maintenance, whatever the switches say.
     */
    enum class Command : uint8_t {
        EXPLORE = 0,
        SOLVE = 1,
        CALIBRATE = 2,
        SAVE = 3,
        RESET = 4,
        STOP = 5,
        RESUME = 6,
        MAINTAIN = 7,
    };

    /**
     * @brief What made the robot stop, as the fault variable of the pool shows it.
     */
    enum class Fault : uint8_t {
        NONE = 0,
        CRASH = 1,
        SATURATION = 2,
        IMU = 3,
        STOPPED = 4,
    };

    /**
     * @brief Construct a new Micras object.
     *
     * @note The estimate of the pose starts where a run starts, in the start cell, rather than at
     * the corner of the maze: the robot is placed there, and that is where a run expects it.
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
     * @brief Stop the robot, turning its sensors and actuators off.
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
     * @brief Send an event to the interface.
     *
     * @param event The event to send.
     */
    void send_event(Interface::Event event);

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
     * @brief Take the procedure the next maintenance runs.
     *
     * @note The one the MAINTAIN command asked for, which is forgotten once taken, and otherwise the
     * one the switches select.
     *
     * @return The procedure.
     */
    Maintenance take_maintenance();

    /**
     * @brief Check if the link asked the robot to stop.
     *
     * @note The request is forgotten once the robot is stopped, in idle or in the error state.
     *
     * @return True if the robot has to stop.
     */
    bool is_stop_requested() const;

    /**
     * @brief Leave the error state, turning off the LED it turned on.
     */
    void leave_error();

    /**
     * @brief Turn the sensors on for the check of the sensors.
     */
    void start_sensor_check();

    /**
     * @brief Start the check of the polarity of the motors and the encoders.
     */
    void start_polarity_check();

    /**
     * @brief Advance the check of the polarity by one iteration.
     *
     * @return True if every step of the check has been driven.
     */
    bool check_polarity();

    /**
     * @brief Turn the wall sensors on to measure their offsets, once they settle.
     */
    void start_offset_calibration();

    /**
     * @brief Advance the calibration of the offsets of the wall sensors by one iteration.
     *
     * @note Once every sensor is measured, the offsets whose spread is small enough are kept and
     * saved to the flash memory.
     *
     * @return True once the calibration has finished.
     */
    bool calibrate_offsets();

    /**
     * @brief Start the check of the crosstalk of the wall sensors, with every emitter off.
     */
    void start_crosstalk_check();

    /**
     * @brief Advance the check of the crosstalk by one iteration.
     *
     * @return True once every mode has been lit for its whole duration.
     */
    bool check_crosstalk();

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
     * @brief Save the maze and the calibrations to the non-volatile storage.
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
     * @brief Start the calibration of the pair of wall sensors that is next in line.
     */
    void start_calibration();

    /**
     * @brief Check on the calibration of the wall sensors.
     *
     * @note A sensor whose readings spread over the maximum keeps the reference it had. Once both
     * pairs are done, the references are saved to the flash memory.
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
     * @note A valid scale within its range is used at once and saved to the flash memory, with the
     * motors stopped, since saving stalls the loop for seconds.
     *
     * @return True if the calibration has finished.
     */
    bool calibrate_gyroscope();

    /**
     * @brief Run a command that arrived over the link.
     *
     * @param code Command to run.
     * @param argument Argument of the command.
     * @return Whether the command ran.
     */
    comm::CommandResult handle_command(uint8_t code, uint32_t argument) override;

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
     * @note The reset flags are those of the reset controller at boot, and the previous trace the
     * mark the program left before that reset, one of Trace, which tells where a reset by the
     * watchdog stopped it. The state is the id of the state machine's current state, one of State. The
     * initialization status has one bit per check of check_initialization that failed, in the order
     * of InitCheck, so a robot that boots into the error state says why. The motor command is the
     * linear and angular share of the supply last applied, in percent. The wall flags hold whether
     * each sensor's reading is valid in the four lowest bits, whether it is blind in the next four
     * and whether it is saturated in the four after those.
     * The crosstalk mode is the one the check of the crosstalk lights, see crosstalk_modes.
     *
     * @note Some of what is worth watching is computed on the way out of its owner: the battery is
     * scaled into volts, the deviations of the estimate come out of its covariance. Publishing means
     * copying those into somewhere that stays put. The route time is zero while no route is planned.
     * The restarts of the converters are counted by the HAL, in a static member. The failed saves are
     * counted as they fail. The results of the maintenance procedures are copied once, when a
     * procedure ends.
     */
    struct Telemetry {
        uint8_t                                        state{};
        uint16_t                                       init_status{};
        uint32_t                                       reset_flags{};
        uint32_t                                       previous_trace{};
        std::array<float, 2>                           motor_command{};
        std::array<float, nav::number_of_wall_sensors> wall_intensities{};
        uint16_t                                       wall_flags{};
        uint8_t                                        crosstalk_mode{};
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
        std::array<float, nav::number_of_wall_sensors> wall_offsets{};
        bool                                           identification_valid{};
        float                                          breakaway_voltage{};
        nav::DriveIdentification::Axis                 linear_drive{};
        nav::DriveIdentification::Axis                 angular_drive{};
        float                                          torque_constant{};
        float                                          resistance{};
        float                                          yaw_inertia{};
        bool                                           gyroscope_scale_valid{};
        float                                          gyroscope_scale{};
    };

    /**
     * @brief Marks the program leaves where a reset would stop it, which the next boot publishes.
     */
    enum class Trace : uint32_t {
        NONE = 0,
        LOOP = 1,
        SAVE_STARTED = 2,
        SAVE_WRITING = 3,
        SAVE_WRITTEN = 4,
        SAVE_DONE = 5,
    };

    /**
     * @brief Checks of the initialization, as the bits of the initialization status.
     */
    enum class InitCheck : uint8_t {
        WATCHDOG_RESET = 0,
        CPU_FREQUENCY = 1,
        FAN = 2,
        LOCOMOTION = 3,
        TORQUE_SENSORS = 4,
        ARGB = 5,
        BUZZER = 6,
        IMU = 7,
        ROTARY_SENSOR_LEFT = 8,
        ROTARY_SENSOR_RIGHT = 9,
        WALL_SENSORS = 10,
    };

    /**
     * @brief Run every check of the initialization.
     *
     * @return One bit per failed check, in the order of InitCheck, so zero if every check passed.
     */
    uint16_t get_init_status() const;

    /**
     * @brief Use the calibrations the flash memory holds wherever the configuration still holds the
     * value they replaced.
     */
    void apply_calibration();

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
     * @brief Copy what is worth watching and has no stable address to the telemetry.
     */
    void publish();

    /**
     * @brief Forget the counts of the faults and the last fault, before the robot starts to move.
     */
    void clear_faults();

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
    // The VBAT pin of the v1 board does not reach the battery, so it is not measured
    // proxy::Battery       battery{battery_config};
    proxy::Fan        fan{fan_config};
    proxy::Locomotion locomotion{locomotion_config};
    proxy::Storage    maze_storage{maze_storage_config};
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
     *
     * @note The wall sensors come before the torque sensors: the converter of the wall sensors is
     * the master of the pair it forms with the one of the torque sensors, and its setup is refused
     * once the other one is converting.
     */
    ///@{
    proxy::Imu           imu{imu_config};
    proxy::RotarySensor  rotary_sensor_left{rotary_sensor_left_config};
    proxy::RotarySensor  rotary_sensor_right{rotary_sensor_right_config};
    proxy::WallSensors   wall_sensors{wall_sensors_config};
    proxy::TorqueSensors torque_sensors{torque_sensors_config};
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
    CalibrateOffsetsState   calibrate_offsets_state{State::CALIBRATE_OFFSETS, *this};
    CheckSensorsState       check_sensors_state{State::CHECK_SENSORS, *this};
    CheckPolarityState      check_polarity_state{State::CHECK_POLARITY, *this};
    CheckCrosstalkState     check_crosstalk_state{State::CHECK_CROSSTALK, *this};
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
     * @brief Time the samples are stamped with, in microseconds.
     *
     * @note Counted in loop periods, so that it wraps around 2^32 microseconds as the link expects.
     * The cycle counter wraps every 7.8 s at 550 MHz.
     */
    uint32_t telemetry_time_us{};

    /**
     * @brief Finite state machine for the robot.
     */
    core::TFsm<std::to_underlying(State::NUMBER_OF_STATES)> fsm{std::to_underlying(State::INIT)};

    /**
     * @brief Measurements of the current iteration.
     */
    nav::Measurements measurements{};

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
     * @brief Procedure the MAINTAIN command asked for, until the next maintenance takes it.
     */
    std::optional<Maintenance> requested_maintenance;

    /**
     * @brief Whether the link asked the robot to stop, until the robot is stopped.
     */
    bool stop_requested{};

    /**
     * @brief Step of the check of the polarity being driven, and the time it has been driven for.
     */
    ///@{
    uint8_t polarity_step{};
    float   polarity_step_time{};
    ///@}

    /**
     * @brief Time the current mode of the check of the crosstalk has been lit for.
     */
    float crosstalk_mode_time{};

    /**
     * @brief Time the wall sensors have been on for during the calibration of their offsets, and
     * whether the measurement started.
     */
    ///@{
    float offset_time{};
    bool  offset_measuring{};
    ///@}

    /**
     * @brief Calibrations measured on the robot, which the flash memory keeps with the maze.
     */
    CalibrationRecord calibration_record;

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
     * @brief Longest run of iterations without a new sample of the inertial measurement unit since
     * the last run or procedure that moves started.
     */
    uint16_t longest_imu_silence{};

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
