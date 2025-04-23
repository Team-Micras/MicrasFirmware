/**
 * @file
 */

#include <bit>
#include <numbers>

#include "micras/proxy/rotary_sensor.hpp"

namespace micras::proxy {
RotarySensor::RotarySensor(const Config& config) :
    spi{config.spi}, encoder{config.encoder}, crc{config.crc}, resolution{config.resolution} {
    CommandFrame command_frame{};
    DataFrame    data_frame{};

    command_frame.fields.address = Registers::disable_addr;
    data_frame.fields.data = config.registers.disable.raw;
    this->write_register(command_frame, data_frame);

    command_frame.fields.address = Registers::zposm_addr;
    data_frame.fields.data = config.registers.zposm.raw;
    this->write_register(command_frame, data_frame);

    command_frame.fields.address = Registers::zposl_addr;
    data_frame.fields.data = config.registers.zposl.raw;
    this->write_register(command_frame, data_frame);

    command_frame.fields.address = Registers::settings1_addr;
    data_frame.fields.data = config.registers.settings1.raw;
    this->write_register(command_frame, data_frame);

    command_frame.fields.address = Registers::settings2_addr;
    data_frame.fields.data = config.registers.settings2.raw;
    this->write_register(command_frame, data_frame);

    command_frame.fields.address = Registers::settings3_addr;
    data_frame.fields.data = config.registers.settings3.raw;
    this->write_register(command_frame, data_frame);

    command_frame.fields.address = Registers::ecc_addr;
    data_frame.fields.data = config.registers.ecc.raw;
    this->write_register(command_frame, data_frame);
}

float RotarySensor::get_position() const {
    return encoder.get_counter() * 2.0F * std::numbers::pi_v<float> / this->resolution;
}

uint16_t RotarySensor::read_register(uint16_t address) {
    CommandFrame    command_frame = {{.crc = 0, .address = address, .rw = 1, .do_not_care = 0}};
    const DataFrame data_frame{};

    command_frame.fields.crc = this->crc.calculate(&command_frame.raw, 2) ^ 0xFF;

    while (not this->spi.select_device()) { }
    this->spi.transmit({std::bit_cast<uint8_t*>(&command_frame.raw), 3});
    this->spi.unselect_device();

    while (not this->spi.select_device()) { }
    this->spi.receive({std::bit_cast<uint8_t*>(&data_frame.raw), 3});
    this->spi.unselect_device();

    return data_frame.fields.data;
}

void RotarySensor::write_register(CommandFrame& command_frame, DataFrame& data_frame) {
    command_frame.fields.crc = this->crc.calculate(&command_frame.raw, 2) ^ 0xFF;
    data_frame.fields.crc = this->crc.calculate(&data_frame.raw, 2) ^ 0xFF;

    while (not this->spi.select_device()) { }
    this->spi.transmit({std::bit_cast<uint8_t*>(&command_frame.raw), 3});
    this->spi.unselect_device();

    while (not this->spi.select_device()) { }
    this->spi.transmit({std::bit_cast<uint8_t*>(&data_frame.raw), 3});
    this->spi.unselect_device();
}
}  // namespace micras::proxy
