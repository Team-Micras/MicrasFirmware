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
RotarySensor::RotarySensor(Config& config) :
    spi{config.spi}, encoder{config.encoder}, crc{config.crc}, resolution{config.resolution} {
    CommandFrame command_frame;
    DataFrame data_frame;

    command_frame.address = Registers::disable_addr;
    data_frame.data = config.registers.disable.raw;
    this->write_register(command_frame, data_frame);

    command_frame.address = Registers::zposm_addr;
    data_frame.data = config.registers.zposm.raw;
    this->write_register(command_frame, data_frame);

    command_frame.address = Registers::zpos_addr;
    data_frame.data = config.registers.zpos.raw;
    this->write_register(command_frame, data_frame);

    command_frame.address = Registers::settings1_addr;
    data_frame.data = config.registers.settings1.raw;
    this->write_register(command_frame, data_frame);

    command_frame.address = Registers::settings2_addr;
    data_frame.data = config.registers.settings2.raw;
    this->write_register(command_frame, data_frame);

    command_frame.address = Registers::settings3_addr;
    data_frame.data = config.registers.settings3.raw;
    this->write_register(command_frame, data_frame);

    command_frame.address = Registers::ecc_addr;
    data_frame.data = config.registers.ecc.raw;
    this->write_register(command_frame, data_frame);
}

float RotarySensor::get_position() const {
    return encoder.get_counter() * 2 * std::numbers::pi_v<float> / this->resolution;
}

void RotarySensor::write_register(CommandFrame& command_frame, DataFrame& data_frame) {
    while (not this->spi.select_device()) {
        continue;
    }

    command_frame.crc = this->crc.calculate(&command_frame.raw, 2);
    data_frame.crc = this->crc.calculate(&data_frame.raw, 2);

    this->spi.transmit(reinterpret_cast<uint8_t*>(&command_frame.raw), 3);
    this->spi.transmit(reinterpret_cast<uint8_t*>(&data_frame.raw), 3);
    this->spi.unselect_device();
}
}  // namespace proxy
