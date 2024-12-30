/**
 * @file
 */

#ifndef MICRAS_CONTROLLER_HPP
#define MICRAS_CONTROLLER_HPP

#include "constants.hpp"
#include "target.hpp"

namespace micras {
/**
 * @brief Class for controlling the Micras robot.
 */
class MicrasController {
public:
    /**
     * @brief Construct a new Micras Controller object.
     */
    MicrasController();

    /**
     * @brief Update the controller loop of the robot.
     */
    void update();

private:
    /**
     * @brief Enum for the current status of the robot.
     */
    enum Status : uint8_t {
        INIT = 0,       // Initialization of the robot.
        IDLE = 1,       // Waiting for the user to start the robot.
        WAIT = 2,       // Timer for the a predefined next state.
        RUN = 3,        // Running the main algorithm.
        CALIBRATE = 4,  // Calibrating the robot.
        ERROR = 5       // Error state.
    };

    /**
     * @brief Enum for the type of calibration being performed.
     */
    enum CalibrationType : uint8_t {
        SIDE_WALLS = 0,        // Calibrate side walls and front free space detection.
        FRONT_WALL = 1,        // Calibrate front wall detection.
        LEFT_FREE_SPACE = 2,   // Calibrate left free space detection.
        RIGHT_FREE_SPACE = 3,  // Calibrate right free space detection.
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
     * @brief Run the robot loop.
     *
     * @param elapsed_time Time elapsed since the last run in seconds.
     * @return True if the robot has ended the run, false otherwise.
     */
    bool run(float elapsed_time);

    /**
     * @brief Calibrate the robot.
     */
    void calibrate();

    /**
     * @brief Stop the robot.
     */
    void stop();

    /**
     * @brief Current status of the robot.
     */
    Status status{Status::INIT};

    /**
     * @brief Next status of the robot after the end of the wait time.
     */
    Status next_status{Status::INIT};

    /**
     * @brief Timer for the wait status.
     */
    hal::Timer wait_timer;

    /**
     * @brief Timer for the run status.
     */
    hal::Timer loop_timer;

    /**
     * @brief Timer for aligning the robot to the back wall.
     */
    hal::Timer align_back_timer;

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
    nav::Mapping::Action current_action{};

    /**
     * @brief Sensors and actuators.
     */
    ///@{
    proxy::Argb         argb;
    proxy::Battery      battery;
    proxy::Button       button;
    proxy::Buzzer       buzzer;
    proxy::DipSwitch    dip_switch;
    proxy::WallSensors  wall_sensors;
    proxy::Fan          fan;
    proxy::Imu          imu;
    proxy::Led          led;
    proxy::Locomotion   locomotion;
    proxy::RotarySensor rotary_sensor_left;
    proxy::RotarySensor rotary_sensor_right;
    proxy::Storage      maze_storage;
    // proxy::TorqueSensors torque_sensors;
    ///@}

    /**
     * @brief High level objects.
     */
    ///@{
    nav::Odometry    odometry;
    nav::Mapping     mapping;
    nav::LookAtPoint look_at_point;
    nav::GoToPoint   go_to_point;
    ///@}
};
}  // namespace micras

#endif  // MICRAS_CONTROLLER_HPP
