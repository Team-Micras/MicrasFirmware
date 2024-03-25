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
RotarySensor::RotarySensor(Config& config) : spi{config.spi}, encoder{config.encoder}, resolution{config.resolution} {
}

float RotarySensor::get_position() {
    return encoder.get_counter() * 2 * std::numbers::pi_v<float> / this->resolution;
}
}  // namespace proxy
