/**
 * @file
 */

#include <algorithm>
#include <array>
#include <cstdint>

#include "constants.hpp"
#include "micras/hal/adc_dma.hpp"
#include "micras/proxy/button.hpp"
#include "micras/proxy/locomotion.hpp"
#include "micras/proxy/stopwatch.hpp"
#include "target.hpp"
#include "test_core.hpp"

using namespace micras;  // NOLINT(google-build-using-namespace)

namespace {
struct Step {
    float left;
    float right;
};
}  // namespace

static constexpr std::array<Step, 9> steps{{
    {.left = 0.0F, .right = 0.0F},
    {.left = 20.0F, .right = 0.0F},
    {.left = 40.0F, .right = 0.0F},
    {.left = -40.0F, .right = 0.0F},
    {.left = 0.0F, .right = 0.0F},
    {.left = 0.0F, .right = 20.0F},
    {.left = 0.0F, .right = 40.0F},
    {.left = 0.0F, .right = -40.0F},
    {.left = 0.0F, .right = 0.0F},
}};

static constexpr uint32_t step_time_ms{1500};
static constexpr uint32_t settle_time_ms{500};
static constexpr uint32_t hold_time_ms{15000};

// NOLINTBEGIN(*-avoid-c-arrays, cppcoreguidelines-avoid-non-const-global-variables)
static volatile float    test_torque[2];
static volatile float    test_torque_raw[2];
static volatile float    test_current[2];
static volatile float    test_current_raw[2];
static volatile float    test_mean_current[steps.size()][2];
static volatile float    test_min_current[steps.size()][2];
static volatile float    test_max_current[steps.size()][2];
static volatile uint32_t test_run{};
static volatile uint32_t test_step{};
static volatile uint32_t test_sweeps{};
static volatile uint32_t test_restarts{};
static volatile bool     test_initialized{};
static volatile float    test_hold[2];

// NOLINTEND(*-avoid-c-arrays, cppcoreguidelines-avoid-non-const-global-variables)

int main(int argc, char* argv[]) {
    TestCore::init(argc, argv);
    proxy::Locomotion    locomotion{locomotion_config};
    proxy::TorqueSensors torque_sensors{torque_sensors_config};
    proxy::Button        button{button_config};
    proxy::Stopwatch     stopwatch;
    std::array<float, 2> sum{};
    uint32_t             samples{};

    test_initialized = locomotion.was_initialized() and torque_sensors.was_initialized();

    TestCore::loop([&]() {
        button.update();

        if (test_run == 0 and button.get_status() != proxy::Button::Status::NO_PRESS) {
            test_run = 1;
        }

        torque_sensors.update();

        for (uint8_t i = 0; i < 2; i++) {
            test_torque[i] = torque_sensors.get_torque(i);
            test_torque_raw[i] = torque_sensors.get_torque_raw(i);
            test_current[i] = torque_sensors.get_current(i);
            test_current_raw[i] = torque_sensors.get_current_raw(i);
        }

        test_restarts = hal::AdcDma::get_restarts();

        if (test_run == 3) {
            test_run = 4;
            stopwatch.reset_ms();
            locomotion.enable();
            locomotion.set_wheel_command(test_hold[0], test_hold[1]);
        }

        if (test_run == 4 and stopwatch.elapsed_time_ms() >= hold_time_ms) {
            locomotion.stop();
            locomotion.disable();
            test_run = 0;
        }

        if (test_run == 1) {
            test_run = 2;
            test_step = 0;
            samples = 0;
            sum = {};
            stopwatch.reset_ms();
            locomotion.enable();
            locomotion.set_wheel_command(steps.at(0).left, steps.at(0).right);

            for (uint8_t i = 0; i < 2; i++) {
                test_min_current[0][i] = 1.0e9F;
                test_max_current[0][i] = -1.0e9F;
            }
        }

        if (test_run == 2) {
            const uint32_t step = test_step;

            if (stopwatch.elapsed_time_ms() >= settle_time_ms) {
                samples++;

                for (uint8_t i = 0; i < 2; i++) {
                    const float current = torque_sensors.get_current(i);
                    sum.at(i) += current;
                    test_mean_current[step][i] = sum.at(i) / static_cast<float>(samples);

                    const float lowest = test_min_current[step][i];
                    const float highest = test_max_current[step][i];
                    test_min_current[step][i] = std::min(lowest, current);
                    test_max_current[step][i] = std::max(highest, current);
                }
            }

            if (stopwatch.elapsed_time_ms() >= step_time_ms) {
                if (step + 1 >= steps.size()) {
                    locomotion.stop();
                    locomotion.disable();
                    test_run = 0;
                    test_sweeps = test_sweeps + 1;
                } else {
                    test_step = step + 1;
                    samples = 0;
                    sum = {};
                    stopwatch.reset_ms();
                    locomotion.set_wheel_command(steps.at(step + 1).left, steps.at(step + 1).right);

                    for (uint8_t i = 0; i < 2; i++) {
                        test_min_current[step + 1][i] = 1.0e9F;
                        test_max_current[step + 1][i] = -1.0e9F;
                    }
                }
            }
        }

        proxy::Stopwatch::sleep_us(loop_time_us);
    });

    return 0;
}
