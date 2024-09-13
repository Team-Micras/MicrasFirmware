/**
 * @file test_rotary_sensors.cpp
 *
 * @brief Test for the Rotary Sensor class
 *
 * @date 05/2024
 */

#include "test_core.hpp"

using namespace micras;  // NOLINT(google-build-using-namespace)

static volatile float test_left_position{};
static volatile float test_right_position{};

static volatile uint16_t test_resolution_byte{};
static volatile uint16_t test_decimal_byte{};

int main(int argc, char* argv[]) {
    TestCore::init(argc, argv);
    proxy::RotarySensor rotary_sensor_left{rotary_sensor_left_config};
    proxy::RotarySensor rotary_sensor_right{rotary_sensor_right_config};

    proxy::RotarySensor::CommandFrame command_frame{};
    proxy::RotarySensor::DataFrame    data_frame{};

    command_frame.fields.address = proxy::RotarySensor::Registers::settings2_addr;
    data_frame.fields.data = rotary_sensor_right_config.registers.settings2.raw;
    rotary_sensor_right.write_register(command_frame, data_frame);

    command_frame.fields.address = proxy::RotarySensor::Registers::settings3_addr;
    data_frame.fields.data = rotary_sensor_right_config.registers.settings3.raw;
    rotary_sensor_right.write_register(command_frame, data_frame);

    test_decimal_byte = rotary_sensor_right.read_register(proxy::RotarySensor::Registers::settings2_addr) & (1 << 5);
    test_resolution_byte = rotary_sensor_right.read_register(proxy::RotarySensor::Registers::settings3_addr) >> 5;

    TestCore::loop([&rotary_sensor_left, &rotary_sensor_right]() {
        test_left_position = rotary_sensor_left.get_position();
        test_right_position = rotary_sensor_right.get_position();
    });

    return 0;
}
