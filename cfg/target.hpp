/**
 * @file target.hpp
 *
 * @brief Target specific configuration
 *
 * @date 03/2024
 */

#ifndef __TARGET_HPP__
#define __TARGET_HPP__

#include "proxy/button.hpp"
#include "proxy/buzzer.hpp"
#include "proxy/current_sensors.hpp"
#include "proxy/dip_switch.hpp"
#include "proxy/distance_sensors.hpp"
#include "proxy/fan.hpp"
#include "proxy/imu.hpp"
#include "proxy/led.hpp"
#include "proxy/locomotion.hpp"

extern proxy::Button::Config button_config;
extern proxy::Buzzer::Config buzzer_config;
extern proxy::CurrentSensors<2>::Config current_sensors_config;
extern proxy::DipSwitch<4>::Config dip_switch_config;
extern proxy::DistanceSensors<4>::Config distance_sensors_config;
extern proxy::Fan::Config fan_config;
extern proxy::Imu::Config imu_config;
extern proxy::Led::Config led_config;
extern proxy::Locomotion::Config locomotion_config;

#endif // __TARGET_HPP__
