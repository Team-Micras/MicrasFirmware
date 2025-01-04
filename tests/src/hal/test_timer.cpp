/**
 * @file
 */

#include "test_core.hpp"

using namespace micras;  // NOLINT(google-build-using-namespace)

static volatile uint32_t test_timer_value{};  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

int main(int argc, char* argv[]) {
    TestCore::init(argc, argv);
    hal::Timer timer{timer_config};

    TestCore::loop([&timer]() {
        test_timer_value += timer.elapsed_time_us();
        timer.reset_us();
        hal::Timer::sleep_ms(2);
    });

    return 0;
}
