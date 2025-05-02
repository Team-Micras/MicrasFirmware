/**
 * @file
 */

#ifndef MICRAS_HPP
#define MICRAS_HPP

#include "constants.hpp"
#include "micras/core/fsm.hpp"
#include "micras/interface.hpp"
#include "target.hpp"

namespace micras {
/**
 * @brief Class for controlling the Micras robot.
 */
class Micras {
public:
    /**
     * @brief Enum for the current status of the robot.
     */
    enum State : uint8_t {
        INIT = 0,                // Initialization of the robot.
        IDLE = 1,                // Waiting for the user to start the robot.
        WAIT_FOR_RUN = 2,        // Timer for entering the RUN state.
        RUN = 3,                 // Running the main algorithm.
        WAIT_FOR_CALIBRATE = 4,  // Timer for entering the CALIBRATE state.
        CALIBRATE = 5,           // Calibrating the robot.
        ERROR = 6                // Error state.
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

private:
    /**
     * @brief Enum for the type of calibration being performed.
     */
    enum CalibrationType : uint8_t {
        SIDE_WALLS = 0,  // Calibrate side walls and front free space detection.
        FRONT_WALL = 1,  // Calibrate front wall detection.
    };

    /**
     * @brief Sensors and actuators.
     */
    ///@{
    proxy::Battery    battery{battery_config};
    proxy::Fan        fan{fan_config};
    proxy::Locomotion locomotion{locomotion_config};
    proxy::Stopwatch  loop_stopwatch{stopwatch_config};
    proxy::Storage    maze_storage{maze_storage_config};
    // proxy::TorqueSensors torque_sensors{torque_sensors_config};
    ///@}

    /**
     * @brief Interface proxies with the external world.
     */
    ///@{
    std::shared_ptr<proxy::Argb>      argb;
    std::shared_ptr<proxy::Button>    button;
    std::shared_ptr<proxy::Buzzer>    buzzer;
    std::shared_ptr<proxy::DipSwitch> dip_switch;
    std::shared_ptr<proxy::Led>       led;
    ///@}

    /**
     * @brief Sensors shared with nav.
     */
    ///@{
    std::shared_ptr<proxy::Imu>          imu;
    std::shared_ptr<proxy::RotarySensor> rotary_sensor_left;
    std::shared_ptr<proxy::RotarySensor> rotary_sensor_right;
    std::shared_ptr<proxy::WallSensors>  wall_sensors;
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
     * @brief Finite state machine for the robot.
     */
    core::FSM fsm{State::INIT};

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
     * @brief Current type of calibration being performed.
     */
    CalibrationType calibration_type{CalibrationType::SIDE_WALLS};

    /**
     * @brief Current action of the robot.
     */
    std::shared_ptr<nav::Action> current_action;

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
