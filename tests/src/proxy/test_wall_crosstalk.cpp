/**
 * @file
 */

#include <array>
#include <cstdint>

#include "micras/hal/pwm.hpp"
#include "micras/proxy/stopwatch.hpp"
#include "micras/proxy/wall_sensors.hpp"
#include "target.hpp"
#include "test_core.hpp"

using namespace micras;  // NOLINT(google-build-using-namespace)

static constexpr uint8_t  number_of_sensors{4};
static constexpr uint8_t  number_of_modes{number_of_sensors + 2};
static constexpr uint32_t settle_time_ms{300};
static constexpr uint32_t mode_time_ms{1300};

// NOLINTBEGIN(*-avoid-c-arrays, cppcoreguidelines-avoid-non-const-global-variables)
static volatile float    test_intensity[number_of_modes][number_of_sensors];
static volatile float    test_dark[number_of_modes][number_of_sensors];
static volatile uint32_t test_samples[number_of_modes];
static volatile uint32_t test_mode{};
static volatile uint32_t test_cycles{};

// NOLINTEND(*-avoid-c-arrays, cppcoreguidelines-avoid-non-const-global-variables)

int main(int argc, char* argv[]) {
    TestCore::init(argc, argv);
    proxy::WallSensors wall_sensors{wall_sensors_config};

    std::array<hal::Pwm, number_of_sensors> emitters{{
        hal::Pwm{std::get<0>(wall_sensors_config.led_pwms)},
        hal::Pwm{std::get<1>(wall_sensors_config.led_pwms)},
        hal::Pwm{std::get<2>(wall_sensors_config.led_pwms)},
        hal::Pwm{std::get<3>(wall_sensors_config.led_pwms)},
    }};

    proxy::Stopwatch                     stopwatch;
    std::array<float, number_of_sensors> intensity_sum{};
    std::array<float, number_of_sensors> dark_sum{};
    uint32_t                             samples{};
    uint32_t                             mode{};

    auto apply = [&emitters](uint32_t new_mode) {
        for (uint8_t i = 0; i < number_of_sensors; i++) {
            const bool lit = new_mode == number_of_modes - 1 or new_mode == static_cast<uint32_t>(i) + 1;
            emitters.at(i).set_duty_cycle(lit ? wall_sensors_config.emitter_duty_cycle : 0.0F);
        }
    };

    apply(mode);

    TestCore::loop([&]() {
        wall_sensors.update();

        if (stopwatch.elapsed_time_ms() >= settle_time_ms and wall_sensors.get_reading(0).is_new) {
            samples++;

            for (uint8_t i = 0; i < number_of_sensors; i++) {
                intensity_sum.at(i) += wall_sensors.get_intensity(i);
                dark_sum.at(i) += wall_sensors.get_reading(i).dark;
                test_intensity[mode][i] = intensity_sum.at(i) / static_cast<float>(samples);
                test_dark[mode][i] = dark_sum.at(i) / static_cast<float>(samples);
            }

            test_samples[mode] = samples;
        }

        if (stopwatch.elapsed_time_ms() >= mode_time_ms) {
            mode = (mode + 1) % number_of_modes;

            if (mode == 0) {
                test_cycles = test_cycles + 1;
            }

            test_mode = mode;
            samples = 0;
            intensity_sum = {};
            dark_sum = {};
            apply(mode);
            stopwatch.reset_ms();
        }
    });

    return 0;
}
