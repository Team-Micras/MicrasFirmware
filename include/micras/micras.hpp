/**
 * @file
 */

#ifndef MICRAS_HPP
#define MICRAS_HPP

#include "constants.hpp"
#include "micras/core/fsm.hpp"
#include "target.hpp"

namespace micras {
/**
 * @brief Class for controlling the Micras robot.
 */
class Micras {
public:
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
     * @brief Run the main algorithm of the robot.
     *
     * @param elapsed_time The elapsed time since the last update.
     * @return True if the robot is still running, false otherwise.
     */
    bool run(float elapsed_time);

    /**
     * @brief Stop the robot.
     */
    void stop();

private:
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
     * @brief Enum for the type of calibration being performed.
     */
    enum CalibrationType : uint8_t {
        SIDE_WALLS = 0,  // Calibrate side walls and front free space detection.
        FRONT_WALL = 1,  // Calibrate front wall detection.
    };

    /**
     * @brief Enum for the dip switch definitions.
     */
    enum Switch : uint8_t {
        DIAGONAL = 0,  // Whether the robot will be able to move in diagonal paths.
        FAN = 1,       // Turn the fan on.
        STOP = 2,      // Whether the robot will stop at each intersection when solving the maze.
        TURBO = 3,     // Increase the robot speed.
    };

    /**
     * @brief Sensors and actuators.
     */
    ///@{
    proxy::Argb       argb{argb_config};
    proxy::Battery    battery{battery_config};
    proxy::Button     button{button_config};
    proxy::Buzzer     buzzer{buzzer_config};
    proxy::DipSwitch  dip_switch{dip_switch_config};
    proxy::Fan        fan{fan_config};
    proxy::Led        led{led_config};
    proxy::Locomotion locomotion{locomotion_config};
    proxy::Stopwatch  loop_stopwatch{stopwatch_config};
    proxy::Storage    maze_storage{maze_storage_config};
    // proxy::TorqueSensors torque_sensors{torque_sensors_config};
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

    /**
     * @brief Declare states as friend classes.
     */
    ///@{
    friend class CalibrateState;
    friend class ErrorState;
    friend class IdleState;
    friend class InitState;
    friend class RunState;
    friend class WaitState;
    ///@}
};
}  // namespace micras

#endif  // MICRAS_HPP
