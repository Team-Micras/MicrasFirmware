/**
 * @file
 */

#include <cstdint>

#include "micras/proxy/button.hpp"
#include "micras/proxy/fan.hpp"
#include "micras/proxy/stopwatch.hpp"
#include "target.hpp"
#include "test_core.hpp"

using namespace micras;  // NOLINT(google-build-using-namespace)

// NOLINTBEGIN(cppcoreguidelines-avoid-non-const-global-variables)
static volatile float    test_fan_speed{};
static volatile float    test_target_speed{50.0F};
static volatile uint32_t test_hold_time_ms{3000};
static volatile uint32_t test_run{};
static volatile uint32_t test_cycles{};
static volatile uint32_t test_faults{};
static volatile bool     test_fault{};
static volatile bool     test_initialized{};

// NOLINTEND(cppcoreguidelines-avoid-non-const-global-variables)

int main(int argc, char* argv[]) {
    TestCore::init(argc, argv);
    proxy::Button button{button_config};
    proxy::Fan    fan{fan_config};

    test_initialized = fan.was_initialized();

    auto watch = [&fan]() {
        test_fan_speed = fan.update();
        test_fault = fan.check_fault();

        if (test_fault) {
            test_faults = test_faults + 1;
        }
    };

    TestCore::loop([&button, &fan, &watch]() {
        while (test_run == 0 and button.get_status() == proxy::Button::Status::NO_PRESS) {
            button.update();
            watch();
        }

        test_run = 1;
        fan.set_speed(test_target_speed);

        while (not fan.is_at_speed()) {
            watch();
        }

        proxy::Stopwatch stopwatch;

        while (stopwatch.elapsed_time_ms() < test_hold_time_ms) {
            watch();
        }

        fan.set_speed(0.0F);

        while (not fan.is_at_speed()) {
            watch();
        }

        test_cycles = test_cycles + 1;
        test_run = 0;
    });

    return 0;
}
