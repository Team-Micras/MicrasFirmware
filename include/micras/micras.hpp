/**
 * @file
 */

#ifndef MICRAS_HPP
#define MICRAS_HPP

#include <cstdint>
#include <memory>
#include <utility>

#include "constants.hpp"
#include "micras/comm/link.hpp"
#include "micras/comm/trace.hpp"
#include "micras/core/fsm.hpp"
#include "micras/core/variable_pool.hpp"
#include "micras/interface.hpp"
#include "target.hpp"

namespace micras {
/**
 * @brief Class for controlling the Micras robot.
 */
class Micras : public comm::ICommandHandler {
public:
    /**
     * @brief Enum for the current status of the robot.
     */
    enum class State : uint8_t {
        INIT = 0,                // Initialization of the robot.
        IDLE = 1,                // Waiting for the user to start the robot.
        WAIT_FOR_RUN = 2,        // Timer for entering the RUN state.
        RUN = 3,                 // Running the main algorithm.
        WAIT_FOR_CALIBRATE = 4,  // Timer for entering the CALIBRATE state.
        CALIBRATE = 5,           // Calibrating the robot.
        ERROR = 6,               // Error state.
        NUMBER_OF_STATES = 7
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
        TRACE_TRIGGER = 5,
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
     * @brief Calibrate the robot.
     *
     * @return True if the calibration is finished, false otherwise.
     */
    bool calibrate();

    /**
     * @brief Prepare the robot for the next run.
     */
    void prepare();

    /**
     * @brief Run the main algorithm of the robot.
     *
     * @return True if the robot is still running, false otherwise.
     */
    bool run();

    /**
     * @brief Stop the robot.
     */
    void stop();

    /**
     * @brief Turn on the sensors and reset the odometry.
     */
    void init();

    /**
     * @brief Reset the robot to its initial state.
     */
    void reset();

    /**
     * @brief Use the imu to check if the robot crashed.
     *
     * @return True if the robot crashed, false otherwise.
     */
    bool check_crash() const;

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
     * @brief Check if the robot was correctly initialized.
     *
     * @return True if the initialization was successful, false otherwise.
     */
    bool check_initialization() const;

    /**
     * @brief Send an event to the interface.
     *
     * @param event The event to send.
     */
    void send_event(Interface::Event event);

    /**
     * @brief Get the value of an event and reset it.
     *
     * @param event The event to get.
     * @return True if the event happened, false otherwise.
     */
    bool acknowledge_event(Interface::Event event);

    /**
     * @brief Get the value of an event without reseting it.
     *
     * @param event The event to get.
     * @return True if the event happened, false otherwise.
     */
    bool peek_event(Interface::Event event) const;

    /**
     * @brief Save the best route to the non-volatile storage.
     */
    void save_best_route();

    /**
     * @brief Load the best route from the non-volatile storage.
     */
    void load_best_route();

    /**
     * @brief Handle events from the interface.
     */
    void handle_events();

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

private:
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
        SIDE_WALLS = 0,  // Calibrate side walls and front free space detection.
        FRONT_WALL = 1,  // Calibrate front wall detection.
    };

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
    proxy::Stopwatch     loop_stopwatch;
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
     * @brief Sensors shared with nav.
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
    nav::ActionQueuer    action_queuer;
    nav::Maze            maze;
    nav::Odometry        odometry;
    nav::SpeedController speed_controller;
    nav::FollowWall      follow_wall;
    ///@}

    /**
     * @brief Every variable exposed to the storage and to the communication link.
     */
    core::TVariablePool<max_variables> variables;

    /**
     * @brief Full rate capture, and the session that arms and reads it out.
     */
    ///@{
    comm::Trace trace;
    comm::Link  link;
    ///@}

    /**
     * @brief Free running clock the samples are stamped with.
     */
    proxy::Stopwatch telemetry_stopwatch;

    /**
     * @brief Finite state machine for the robot.
     */
    core::TFsm<std::to_underlying(State::NUMBER_OF_STATES)> fsm{std::to_underlying(State::INIT)};

    /**
     * @brief Class for controlling the interface with the external world.
     */
    Interface interface;

    /**
     * @brief Time elapsed since the last loop in seconds.
     */
    float elapsed_time{};

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
     * @brief Current action of the robot.
     */
    std::shared_ptr<nav::Action> current_action;

    /**
     * @brief Longest control loop body observed since the last reset, in microseconds.
     *
     * @note Not acted on, but the only way to know how much of the loop budget is actually used,
     * which every performance decision depends on. Read it with a debugger or a variable monitor.
     */
    uint32_t worst_loop_time_us{};

    /**
     * @brief Current pose of the robot in the maze.
     */
    nav::GridPose grid_pose{};

    /**
     * @brief Current pose of the robot relative to the current action.
     */
    nav::RelativePose action_pose;

    /**
     * @brief Flag for when the robot has finished an objective.
     */
    bool finished{};

    /**
     * @brief Current desired linear and angular speeds of the robot.
     */
    nav::Twist desired_speeds{};

    /**
     * @brief Last response of the speed controller to the left motor.
     */
    float left_response{};

    /**
     * @brief Last response of the speed controller to the right motor.
     */
    float right_response{};

    /**
     * @brief Last feed forward command to the left motor.
     */
    float left_ff{};

    /**
     * @brief Last feed forward command to the right motor.
     */
    float right_ff{};
};
}  // namespace micras

#endif  // MICRAS_HPP
