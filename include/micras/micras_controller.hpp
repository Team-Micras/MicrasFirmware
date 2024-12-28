/**
 * @file
 */

#ifndef MICRAS_CONTROLLER_HPP
#define MICRAS_CONTROLLER_HPP

#include "constants.hpp"
#include "target.hpp"

namespace micras {
/**
 * @brief Class for controlling the Micras robot
 */
class MicrasController {
public:
    /**
     * @brief Constructor for the MicrasController class
     */
    MicrasController();

    /**
     * @brief Update the controller loop of the robot
     */
    void update();

private:
    /**
     * @brief Enum for the current status of the robot
     */
    enum Status : uint8_t {
        INIT = 0,
        IDLE = 1,
        WAIT = 2,
        RUN = 3,
        CALIBRATE = 4,
        ERROR = 5
    };

    /**
     * @brief Enum for the type of calibration being performed
     */
    enum CalibrationType : uint8_t {
        SIDE_WALLS = 0,
        FRONT_WALL = 1,
        LEFT_FREE_SPACE = 2,
        RIGHT_FREE_SPACE = 3,
    };

    /**
     * @brief Enum for the switch definitions
     */
    enum Switch : uint8_t {
        DIAGONAL = 0,
        FAN = 1,
        STOP = 2,
        TURBO = 3,
    };

    /**
     * @brief Run the robot loop
     *
     * @param elapsed_time Time elapsed since the last run
     * @return true if the robot has ended the run, false otherwise
     */
    bool run(float elapsed_time);

    /**
     * @brief Calibrate the robot
     */
    void calibrate();

    /**
     * @brief Stop the robot
     */
    void stop();

    /**
     * @brief Current status of the robot
     */
    Status status{Status::INIT};

    /**
     * @brief Next status of the robot after the end of the wait time
     */
    Status next_status{Status::INIT};

    /**
     * @brief Timer for the wait status
     */
    hal::Timer wait_timer;

    /**
     * @brief Timer for the run status
     */
    hal::Timer loop_timer;

    /**
     * @brief Timer for aligning the robot to the back wall
     */
    hal::Timer align_back_timer;

    /**
     * @brief Current objective of the robot
     */
    core::Objective objective{core::Objective::EXPLORE};

    /**
     * @brief Current type of calibration being performed
     */
    CalibrationType calibration_type{CalibrationType::SIDE_WALLS};

    /**
     * @brief Current action of the robot
     */
    nav::Mapping::Action current_action{};

    /**
     * @brief Sensors and actuators
     */
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

    /**
     * @brief High level objects
     */
    nav::Odometry    odometry;
    nav::Mapping     mapping;
    nav::LookAtPoint look_at_point;
    nav::GoToPoint   go_to_point;
};
}  // namespace micras

#endif  // MICRAS_CONTROLLER_HPP
