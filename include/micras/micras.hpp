/**
 * @file
 */

#ifndef MICRAS_HPP
#define MICRAS_HPP

#include <array>
#include <cstdint>
#include <utility>

#include "constants.hpp"
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
     * @brief Commands the link can ask the robot to run.
     *
     * @note These are edges, not levels: each one happens once, when it arrives. Everything that
     * is a level, like the run profile, is a writable variable instead.
     */
    enum class Command : uint8_t {
        EXPLORE = 0,
        SOLVE = 1,
        CALIBRATE = 2,
        SAVE = 3,
        RESET = 4,
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
     * @brief Get the procedure the switches select for an extra long press of the button.
     *
     * @return The procedure.
     */
    Maintenance get_maintenance() const;

    /**
     * @brief Get the robot ready to move: sensors on, and the fan too if the run uses it.
     */
    void prepare();

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
     * @brief Use the imu to check if the robot crashed.
     *
     * @return True if the robot crashed, false otherwise.
     */
    bool check_crash();

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
     */
    void save_maze();

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
     * @brief Values published over the link that no object holds at a stable address.
     *
     * @note Most of what is worth watching is computed on the way out of its sensor: the yaw rate
     * has the calibration subtracted from it, the battery is scaled into volts. Publishing means
     * copying those into somewhere that stays put.
     */
    struct Telemetry {
        std::array<float, 3> angular_velocity{};
        std::array<float, 3> linear_acceleration{};
        float                battery_voltage{};
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
        SIDE_WALLS = 0,  // Calibrate the sensors that look at the side walls, between two walls.
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
     * @brief Copy what is worth watching to the variables a monitor can read and to the telemetry.
     */
    void publish();

    /**
     * @brief Watchdog, started before anything that could hang.
     */
    proxy::Watchdog watchdog{watchdog_config};

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
     */
    proxy::Stopwatch telemetry_stopwatch;

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
     * write it when they move, the link whenever it likes.
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
     * @note The planning and the saving of the maze take many periods and are counted here too,
     * with the robot stopped, and so are the one or two that flooding the maze takes whenever a
     * search decides a wall. What matters is that it does not grow during a fast run.
     */
    uint32_t missed_ticks{};

    /**
     * @brief Number of iterations in which the motors could not deliver the command.
     */
    uint32_t saturated_iterations{};
};
}  // namespace micras

#endif  // MICRAS_HPP
