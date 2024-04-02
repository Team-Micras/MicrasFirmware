/**
 * @file micras_controller_test.hpp
 *
 * @brief Micras Controller Test class header
 *
 * @date 03/2024
 */

#ifndef MICRAS_CONTROLLER_HPP
#define MICRAS_CONTROLLER_HPP

#include "proxy/argb.hpp"
#include "proxy/battery.hpp"
#include "proxy/button.hpp"
#include "proxy/buzzer.hpp"
#include "proxy/dip_switch.hpp"
#include "proxy/distance_sensors.hpp"
#include "proxy/fan.hpp"
#include "proxy/imu.hpp"
#include "proxy/led.hpp"
#include "proxy/locomotion.hpp"
#include "proxy/rotary_sensor.hpp"
#include "proxy/torque_sensors.hpp"

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
        proxy::Argb<2> argb;
        proxy::Battery battery;
        proxy::Button button;
        proxy::Buzzer buzzer;
        proxy::DipSwitch<4> dip_switch;
        proxy::DistanceSensors<4> distance_sensors;
        proxy::Fan fan;
        proxy::Imu imu;
        proxy::Led led;
        proxy::Locomotion locomotion;
        proxy::RotarySensor rotary_sensor_left;
        proxy::RotarySensor rotary_sensor_right;
        proxy::TorqueSensors<2> torque_sensors;
};

#endif // MICRAS_CONTROLLER_HPP
