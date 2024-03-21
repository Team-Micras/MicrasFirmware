/**
 * @file micras_controller_test.hpp
 *
 * @brief Micras Controller Test class header
 *
 * @date 03/2024
 */

#ifndef __MICRAS_CONTROLLER_HPP__
#define __MICRAS_CONTROLLER_HPP__

#include "proxy/button.hpp"
#include "proxy/buzzer.hpp"
#include "proxy/current_sensors.hpp"
#include "proxy/dip_switch.hpp"
#include "proxy/distance_sensors.hpp"
#include "proxy/fan.hpp"
#include "proxy/imu.hpp"
#include "proxy/led.hpp"
#include "proxy/locomotion.hpp"

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
        proxy::Button button;
        proxy::Buzzer buzzer;
        proxy::CurrentSensors<2> current_sensors;
        proxy::DipSwitch<4> dip_switch;
        proxy::DistanceSensors<4> distance_sensors;
        proxy::Fan fan;
        proxy::Imu imu;
        proxy::Led led;
        proxy::Locomotion locomotion;
};

#endif // __MICRAS_CONTROLLER_HPP__
