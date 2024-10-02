/**
 * @file micras_controller_test.hpp
 *
 * @brief Micras Controller Test class header
 *
 * @date 03/2024
 */

#ifndef MICRAS_CONTROLLER_HPP
#define MICRAS_CONTROLLER_HPP

#include "constants.hpp"
#include "micras/nav/go_to_point.hpp"
#include "micras/nav/look_at_point.hpp"
#include "micras/nav/mapping.hpp"
#include "micras/nav/odometry.hpp"
#include "micras/proxy/argb.hpp"
#include "micras/proxy/battery.hpp"
#include "micras/proxy/button.hpp"
#include "micras/proxy/buzzer.hpp"
#include "micras/proxy/dip_switch.hpp"
#include "micras/proxy/distance_sensors.hpp"
#include "micras/proxy/fan.hpp"
#include "micras/proxy/imu.hpp"
#include "micras/proxy/led.hpp"
#include "micras/proxy/locomotion.hpp"
#include "micras/proxy/rotary_sensor.hpp"
#include "micras/proxy/torque_sensors.hpp"

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
     * @brief Runs the controller loop once
     */
    void run();

private:
    proxy::Argb<2>            argb;
    proxy::Battery            battery;
    proxy::Button             button;
    proxy::Buzzer             buzzer;
    proxy::DipSwitch<4>       dip_switch;
    proxy::DistanceSensors<4> distance_sensors;
    proxy::Fan                fan;
    proxy::Imu                imu;
    proxy::Led                led;
    proxy::Locomotion         locomotion;
    proxy::RotarySensor       rotary_sensor_left;
    proxy::RotarySensor       rotary_sensor_right;
    proxy::TorqueSensors<2>   torque_sensors;

    nav::Odometry                         odometry;
    nav::Mapping<maze_width, maze_height> mapping;
    nav::LookAtPoint                      look_at_point;
    nav::GoToPoint                        go_to_point;

    nav::Mapping<maze_width, maze_height>::Action current_action{};
};
}  // namespace micras

#endif  // MICRAS_CONTROLLER_HPP
