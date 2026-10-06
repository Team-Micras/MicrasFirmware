/**
 * @file
 */

#include <array>
#include <bit>
#include <cstdint>
#include <optional>
#include <vector>

#include "micras/calibration_record.hpp"

namespace micras {
void CalibrationRecord::record(Value& value, float measured, float configured) {
    value = {.measured = measured, .replaced = configured, .present = true};
}

std::optional<float> CalibrationRecord::choose(Value& value, float configured, float min, float max) {
    if (not value.present) {
        return std::nullopt;
    }

    if (std::bit_cast<uint32_t>(value.replaced) != std::bit_cast<uint32_t>(configured) or value.measured < min or
        value.measured > max) {
        value.present = false;
        return std::nullopt;
    }

    return value.measured;
}

template <typename Function>
void CalibrationRecord::for_each(Function function) {
    for (Value& value : this->wall_reference_readings) {
        function(value);
    }

    for (Value& value : this->wall_offsets) {
        function(value);
    }

    function(this->gyroscope_scale);
}

template <typename Function>
void CalibrationRecord::for_each(Function function) const {
    for (const Value& value : this->wall_reference_readings) {
        function(value);
    }

    for (const Value& value : this->wall_offsets) {
        function(value);
    }

    function(this->gyroscope_scale);
}

std::vector<uint8_t> CalibrationRecord::serialize() const {
    std::vector<uint8_t> data{version};
    data.reserve(1 + number_of_values * value_size);

    this->for_each([&data](const Value& value) {
        data.push_back(value.present ? 1 : 0);

        for (const float number : {value.measured, value.replaced}) {
            const auto bits = std::bit_cast<uint32_t>(number);

            for (uint8_t shift = 0; shift < 32; shift += 8) {
                data.push_back(static_cast<uint8_t>(bits >> shift));
            }
        }
    });

    return data;
}

void CalibrationRecord::deserialize(const uint8_t* serial_data, uint16_t size) {
    *this = {};

    if (size != 1 + number_of_values * value_size or serial_data[0] != version) {
        return;
    }

    const uint8_t* cursor = serial_data + 1;

    this->for_each([&cursor](Value& value) {
        value.present = cursor[0] != 0;

        std::array<float*, 2> numbers{&value.measured, &value.replaced};

        for (uint8_t i = 0; i < numbers.size(); i++) {
            uint32_t bits = 0;

            for (uint8_t byte = 0; byte < 4; byte++) {
                bits |= static_cast<uint32_t>(cursor[1 + 4 * i + byte]) << (8 * byte);
            }

            *numbers.at(i) = std::bit_cast<float>(bits);
        }

        cursor += value_size;
    });
}
}  // namespace micras
