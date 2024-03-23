/**
 * @file target.hpp
 *
 * @brief Target specific configuration
 *
 * @date 03/2024
 */

#ifndef __TARGET_HPP__
#define __TARGET_HPP__

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

extern proxy::Battery::Config battery_config;
extern proxy::Button::Config button_config;
extern proxy::Buzzer::Config buzzer_config;
extern proxy::DipSwitch<4>::Config dip_switch_config;
extern proxy::DistanceSensors<4>::Config distance_sensors_config;
extern proxy::Fan::Config fan_config;
extern proxy::Imu::Config imu_config;
extern proxy::Led::Config led_config;
extern proxy::Locomotion::Config locomotion_config;
extern proxy::RotarySensor::Config rotary_sensor_left_config;
extern proxy::RotarySensor::Config rotary_sensor_right_config;
extern proxy::TorqueSensors<2>::Config torque_sensors_config;

#endif // __TARGET_HPP__
