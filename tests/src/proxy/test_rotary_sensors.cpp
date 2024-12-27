/**
 * @file
 */

#include "test_core.hpp"

using namespace micras;  // NOLINT(google-build-using-namespace)

static constexpr float linear_speed{50.0F};

// NOLINTBEGIN(cppcoreguidelines-avoid-non-const-global-variables)
static volatile float test_left_position{};
static volatile float test_right_position{};

static volatile uint16_t test_resolution_byte{};
static volatile uint16_t test_decimal_byte{};

// NOLINTEND(cppcoreguidelines-avoid-non-const-global-variables)

int main(int argc, char* argv[]) {
    TestCore::init(argc, argv);
    proxy::RotarySensor rotary_sensor_left{rotary_sensor_left_config};
    proxy::RotarySensor rotary_sensor_right{rotary_sensor_right_config};
    proxy::Locomotion   locomotion{locomotion_config};
    proxy::Button       button{button_config};
    bool                running{};

    locomotion.enable();

    proxy::RotarySensor::CommandFrame command_frame{};
    proxy::RotarySensor::DataFrame    data_frame{};

    command_frame.fields.address = proxy::RotarySensor::Registers::settings3_addr;
    data_frame.fields.data = rotary_sensor_right_config.registers.settings3.raw;
    rotary_sensor_right.write_register(command_frame, data_frame);

    test_decimal_byte = rotary_sensor_right.read_register(proxy::RotarySensor::Registers::settings2_addr) & (1 << 5);
    test_resolution_byte = rotary_sensor_right.read_register(proxy::RotarySensor::Registers::settings3_addr) >> 5;

    TestCore::loop([&rotary_sensor_left, &rotary_sensor_right, &button, &locomotion, &running]() {
        test_left_position = rotary_sensor_left.get_position();
        test_right_position = rotary_sensor_right.get_position();

        auto button_status = button.get_status();

        if (button_status != proxy::Button::Status::NO_PRESS) {
            running = not running;
            locomotion.set_command(running ? linear_speed : 0.0F, 0.0F);
        }
    });

    return 0;
}
