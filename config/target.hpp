/**
 * @file target.hpp
 *
 * @brief Target specific configuration
 *
 * @date 03/2024
 */

#ifndef __TARGET_HPP__
#define __TARGET_HPP__

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
}  // namespace micras

#endif  // __TARGET_HPP__
