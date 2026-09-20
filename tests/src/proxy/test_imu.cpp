/**
 * @file
 */

#include <cstdint>

#include "constants.hpp"
#include "micras/proxy/imu.hpp"
#include "micras/proxy/stopwatch.hpp"
#include "target.hpp"
#include "test_core.hpp"

using namespace micras;  // NOLINT(google-build-using-namespace)

// NOLINTBEGIN(*-avoid-c-arrays, cppcoreguidelines-avoid-non-const-global-variables)
static volatile float test_angular_velocity[3]{};
static volatile float test_linear_acceleration[3]{};

// NOLINTEND(*-avoid-c-arrays, cppcoreguidelines-avoid-non-const-global-variables)

/**
 * @brief Number of iterations and of new samples, which run apart if the sensor does not deliver
 * its data rate or if a transfer does not fit in an iteration.
 */
// NOLINTBEGIN(cppcoreguidelines-avoid-non-const-global-variables)
static volatile uint32_t test_iterations{};
static volatile uint32_t test_new_samples{};

// NOLINTEND(cppcoreguidelines-avoid-non-const-global-variables)

int main(int argc, char* argv[]) {
    TestCore::init(argc, argv);

    proxy::Imu       imu{imu_config};
    proxy::Argb      argb{argb_config};
    proxy::Stopwatch stopwatch;

    proxy::Stopwatch::sleep_ms(2);

    if (imu.was_initialized()) {
        argb.set_color(proxy::Argb::Colors::green);

    } else {
        argb.set_color(proxy::Argb::Colors::red);

        while (true) { }
    }

    TestCore::loop([&imu, &stopwatch]() {
        stopwatch.reset_us();
        imu.update();

        test_iterations = test_iterations + 1;

        if (imu.is_new()) {
            test_new_samples = test_new_samples + 1;
        }

        test_angular_velocity[0] = imu.get_angular_velocity(proxy::Imu::Axis::X);
        test_angular_velocity[1] = imu.get_angular_velocity(proxy::Imu::Axis::Y);
        test_angular_velocity[2] = imu.get_angular_velocity(proxy::Imu::Axis::Z);

        test_linear_acceleration[0] = imu.get_linear_acceleration(proxy::Imu::Axis::X);
        test_linear_acceleration[1] = imu.get_linear_acceleration(proxy::Imu::Axis::Y);
        test_linear_acceleration[2] = imu.get_linear_acceleration(proxy::Imu::Axis::Z);

        while (stopwatch.elapsed_time_us() < loop_time_us) { }
    });

    return 0;
}
