/**
 * @file
 *
 * @brief RotarySensor for the simulator: no SPI register protocol, the real encoder count.
 *
 * @note Replaces micras_proxy/src/rotary_sensor.cpp. On the robot the proxy writes
 *       the AS5047U's registers over SPI, reads back the ABI resolution and then
 *       counts the ABI output with a timer in encoder mode. Here the resolution is
 *       taken from the same configured registers, as if the read back returned
 *       them, and the count comes from the host encoder the simulator's encoder
 *       device drives. The CRC and SPI members are built, as the header requires,
 *       and left unused.
 */

#include <cstdint>
#include <numbers>
#include <optional>

#include "micras/proxy/rotary_sensor.hpp"

namespace micras::proxy {
RotarySensor::RotarySensor(const Config& config) : spi{config.spi}, encoder{config.encoder}, crc{config.crc} {
    if (not this->spi.was_initialized() or not this->encoder.was_initialized()) {
        return;
    }

    const uint16_t pulses = pulses_per_revolution.at(config.registers.settings3.fields.ABIRES);

    if (pulses == 0) {
        return;
    }

    this->resolution = edges_per_pulse * pulses;
    this->initialized = true;
}

float RotarySensor::get_position() const {
    if (this->resolution == 0) {
        return 0.0F;
    }

    return static_cast<float>(this->encoder.get_counter()) * 2.0F * std::numbers::pi_v<float> /
           static_cast<float>(this->resolution);
}

uint32_t RotarySensor::get_resolution() const {
    return this->resolution;
}

bool RotarySensor::was_initialized() const {
    return this->initialized;
}

// NOLINTBEGIN(readability-convert-member-functions-to-static): the firmware's RotarySensor declares them as members.
std::optional<uint16_t> RotarySensor::read_register(uint16_t /*address*/) {
    return std::nullopt;
}

bool RotarySensor::write_register(uint16_t /*address*/, uint16_t /*data*/) {
    return false;
}

// NOLINTEND(readability-convert-member-functions-to-static)
}  // namespace micras::proxy
