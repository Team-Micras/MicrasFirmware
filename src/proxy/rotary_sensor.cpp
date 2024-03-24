/**
 * @file rotary_sensor.cpp
 *
 * @brief Proxy RotarySensor class source
 *
 * @date 03/2024
 */

#include <numbers>

#include "proxy/rotary_sensor.hpp"

namespace proxy {
RotarySensor::RotarySensor(Config& rotary_sensor_config) :
    spi{rotary_sensor_config.spi}, encoder{rotary_sensor_config.encoder} {
}

float RotarySensor::get_position() {
    return 2 * std::numbers::pi_v<float> * encoder.get_counter() / this->resolution;
}
}  // namespace proxy
