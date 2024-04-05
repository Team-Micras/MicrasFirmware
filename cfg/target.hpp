/**
 * @file target.hpp
 *
 * @brief Target specific configuration
 *
 * @date 03/2024
 */

#ifndef __TARGET_HPP__
#define __TARGET_HPP__

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

extern const proxy::Argb<2>::Config            argb_config;
extern const proxy::Battery::Config            battery_config;
extern const proxy::Button::Config             button_config;
extern const proxy::Buzzer::Config             buzzer_config;
extern const proxy::DipSwitch<4>::Config       dip_switch_config;
extern const proxy::DistanceSensors<4>::Config distance_sensors_config;
extern const proxy::Fan::Config                fan_config;
extern const proxy::Imu::Config                imu_config;
extern const proxy::Led::Config                led_config;
extern const proxy::Locomotion::Config         locomotion_config;
extern const proxy::RotarySensor::Config       rotary_sensor_left_config;
extern const proxy::RotarySensor::Config       rotary_sensor_right_config;
extern const proxy::TorqueSensors<2>::Config   torque_sensors_config;

#endif  // __TARGET_HPP__
