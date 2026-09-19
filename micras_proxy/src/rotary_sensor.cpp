/**
 * @file
 */

#include <array>
#include <cstdint>
#include <numbers>
#include <optional>

#include "micras/proxy/rotary_sensor.hpp"

namespace micras::proxy {
RotarySensor::RotarySensor(const Config& config) : spi{config.spi}, encoder{config.encoder}, crc{config.crc} {
    if (not this->spi.was_initialized() or not this->encoder.was_initialized()) {
        return;
    }

    this->write_register(Registers::disable_addr, config.registers.disable.raw);
    this->write_register(Registers::zposm_addr, config.registers.zposm.raw);
    this->write_register(Registers::zposl_addr, config.registers.zposl.raw);
    this->write_register(Registers::settings1_addr, config.registers.settings1.raw);
    this->write_register(Registers::settings2_addr, config.registers.settings2.raw);
    this->write_register(Registers::settings3_addr, config.registers.settings3.raw);
    this->write_register(Registers::ecc_addr, config.registers.ecc.raw);

    const std::optional<uint32_t> read_back = this->read_resolution();

    if (not read_back.has_value()) {
        return;
    }

    // The scale comes from what the sensor reports, not from what was written to it, so a
    // configuration write that silently failed cannot turn into a wrongly scaled odometry
    this->resolution = read_back.value();
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

std::array<uint8_t, RotarySensor::frame_size> RotarySensor::serialize(uint32_t raw) {
    std::array<uint8_t, frame_size> bytes{
        static_cast<uint8_t>(raw >> 16),
        static_cast<uint8_t>(raw >> 8),
        static_cast<uint8_t>(raw),
    };

    // The sensor defines the CRC over the two most significant bytes of the frame, which are the
    // first two on the wire and the last two in memory on a little endian core
    std::get<2>(bytes) = static_cast<uint8_t>(this->crc.calculate({bytes.data(), frame_size - 1}) ^ 0xFF);

    return bytes;
}

std::optional<uint32_t> RotarySensor::exchange_frame(const std::array<uint8_t, frame_size>& frame) {
    std::array<uint8_t, frame_size> received{};

    if (not this->spi.select_device()) {
        return std::nullopt;
    }

    const bool transferred = this->spi.transmit_receive(frame, received);
    this->spi.unselect_device();

    if (not transferred) {
        return std::nullopt;
    }

    return static_cast<uint32_t>(std::get<0>(received)) << 16 | static_cast<uint32_t>(std::get<1>(received)) << 8 |
           std::get<2>(received);
}

std::optional<uint16_t> RotarySensor::read_register(uint16_t address) {
    const CommandFrame                    command{{.crc = 0, .address = address, .rw = 1, .do_not_care = 0}};
    const std::array<uint8_t, frame_size> command_bytes = this->serialize(command.raw);

    // The sensor answers a command in the frame that follows it, so the command is sent twice: the
    // first transfer carries it and the second clocks the answer out while repeating it
    if (not this->exchange_frame(command_bytes).has_value()) {
        return std::nullopt;
    }

    const std::optional<uint32_t> answer = this->exchange_frame(command_bytes);

    if (not answer.has_value()) {
        return std::nullopt;
    }

    DataFrame data{};
    data.raw = answer.value();

    const std::array<uint8_t, frame_size> expected = this->serialize(answer.value());

    if (std::get<2>(expected) != data.fields.crc or data.fields.error != 0) {
        return std::nullopt;
    }

    return static_cast<uint16_t>(data.fields.data);
}

bool RotarySensor::write_register(uint16_t address, uint16_t data) {
    const CommandFrame command{{.crc = 0, .address = address, .rw = 0, .do_not_care = 0}};
    const DataFrame    value{{.crc = 0, .data = data, .error = 0, .warning = 0}};

    return this->exchange_frame(this->serialize(command.raw)).has_value() and
           this->exchange_frame(this->serialize(value.raw)).has_value();
}

std::optional<uint32_t> RotarySensor::read_resolution() {
    const std::optional<uint16_t> settings3 = this->read_register(Registers::settings3_addr);

    if (not settings3.has_value()) {
        return std::nullopt;
    }

    Registers::Settings3 fields{};
    fields.raw = static_cast<uint8_t>(settings3.value());

    const uint16_t pulses = pulses_per_revolution.at(fields.fields.ABIRES);

    if (pulses == 0) {
        return std::nullopt;
    }

    return edges_per_pulse * pulses;
}
}  // namespace micras::proxy
