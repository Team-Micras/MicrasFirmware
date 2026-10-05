/**
 * @file
 */

#include <array>
#include <cstdint>
#include <numbers>
#include <optional>

#include "micras/proxy/button.hpp"
#include "micras/proxy/locomotion.hpp"
#include "micras/proxy/rotary_sensor.hpp"
#include "micras/proxy/stopwatch.hpp"
#include "target.hpp"
#include "test_core.hpp"

using namespace micras;  // NOLINT(google-build-using-namespace)

static constexpr float    linear_speed{50.0F};
static constexpr uint32_t read_interval_ms{10};
static constexpr uint16_t errfl_addr{0x0001};
static constexpr uint16_t dia_addr{0x3FF5};
static constexpr uint16_t agc_addr{0x3FF9};
static constexpr uint16_t mag_addr{0x3FFD};
static constexpr uint16_t anglecom_addr{0x3FFF};
static constexpr uint16_t angle_mask{0x3FFF};
static constexpr float    angle_range{16384.0F};

// NOLINTBEGIN(*-avoid-c-arrays, cppcoreguidelines-avoid-non-const-global-variables)
static volatile float    test_left_position{};
static volatile float    test_right_position{};
static volatile bool     test_initialized[2];
static volatile uint32_t test_resolution[2];
static volatile float    test_angle[2];
static volatile uint16_t test_errfl[2];
static volatile uint16_t test_dia[2];
static volatile uint16_t test_agc[2];
static volatile uint16_t test_mag[2];
static volatile uint32_t test_reads[2];
static volatile uint32_t test_failed_reads[2];
static volatile float    test_turns[2];
static volatile float    test_counts[2];
static volatile float    test_counts_per_turn[2];

// NOLINTEND(*-avoid-c-arrays, cppcoreguidelines-avoid-non-const-global-variables)

static std::array<int32_t, 2> last_angle{-1, -1};
static std::array<int32_t, 2> unwrapped_angle{};
static std::array<float, 2>   start_position{};

static void read_diagnostics(proxy::RotarySensor& rotary_sensor, uint8_t index) {
    const std::optional<uint16_t> angle = rotary_sensor.read_register(anglecom_addr);
    const std::optional<uint16_t> errfl = rotary_sensor.read_register(errfl_addr);
    const std::optional<uint16_t> dia = rotary_sensor.read_register(dia_addr);
    const std::optional<uint16_t> agc = rotary_sensor.read_register(agc_addr);
    const std::optional<uint16_t> mag = rotary_sensor.read_register(mag_addr);

    if (not angle.has_value() or not errfl.has_value() or not dia.has_value() or not agc.has_value() or
        not mag.has_value()) {
        test_failed_reads[index] = test_failed_reads[index] + 1;
        return;
    }

    test_reads[index] = test_reads[index] + 1;
    test_angle[index] = static_cast<float>(angle.value() & angle_mask) * 2.0F * std::numbers::pi_v<float> / angle_range;
    test_errfl[index] = errfl.value();
    test_dia[index] = dia.value();
    test_agc[index] = agc.value();
    test_mag[index] = mag.value();

    const auto  raw_angle = static_cast<int32_t>(angle.value() & angle_mask);
    const float position = rotary_sensor.get_position();

    if (last_angle.at(index) < 0) {
        start_position.at(index) = position;
    } else {
        int32_t delta = raw_angle - last_angle.at(index);

        if (delta > static_cast<int32_t>(angle_range) / 2) {
            delta -= static_cast<int32_t>(angle_range);
        } else if (delta < -static_cast<int32_t>(angle_range) / 2) {
            delta += static_cast<int32_t>(angle_range);
        }

        unwrapped_angle.at(index) += delta;
    }

    last_angle.at(index) = raw_angle;
    test_turns[index] = static_cast<float>(unwrapped_angle.at(index)) / angle_range;
    test_counts[index] = (position - start_position.at(index)) * static_cast<float>(rotary_sensor.get_resolution()) /
                         (2.0F * std::numbers::pi_v<float>);

    if (test_turns[index] > 0.25F or test_turns[index] < -0.25F) {
        test_counts_per_turn[index] = test_counts[index] / test_turns[index];
    }
}

int main(int argc, char* argv[]) {
    TestCore::init(argc, argv);
    proxy::RotarySensor rotary_sensor_left{rotary_sensor_left_config};
    proxy::RotarySensor rotary_sensor_right{rotary_sensor_right_config};
    proxy::Locomotion   locomotion{locomotion_config};
    proxy::Button       button{button_config};
    proxy::Stopwatch    stopwatch;
    bool                running{};

    test_initialized[0] = rotary_sensor_left.was_initialized();
    test_initialized[1] = rotary_sensor_right.was_initialized();
    test_resolution[0] = rotary_sensor_left.get_resolution();
    test_resolution[1] = rotary_sensor_right.get_resolution();

    TestCore::loop([&]() {
        test_left_position = rotary_sensor_left.get_position();
        test_right_position = rotary_sensor_right.get_position();

        if (stopwatch.elapsed_time_ms() >= read_interval_ms) {
            stopwatch.reset_ms();
            read_diagnostics(rotary_sensor_left, 0);
            read_diagnostics(rotary_sensor_right, 1);
        }

        button.update();

        if (button.get_status() != proxy::Button::Status::NO_PRESS) {
            running = not running;

            if (running) {
                locomotion.enable();
                locomotion.set_command(linear_speed, 0.0F);
            } else {
                locomotion.disable();
                locomotion.stop();
            }
        }
    });

    return 0;
}
