/**
 * @file test_timer.cpp
 *
 * @brief Test for the Timer class
 *
 * @date 05/2024
 */

#include "test_core.hpp"

using namespace micras;  // NOLINT(google-build-using-namespace)

static volatile uint32_t timer_value{};  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

int main(int argc, char* argv[]) {
    TestCore::init(argc, argv);
    hal::Timer timer{timer_config};

    TestCore::loop([&timer]() {
        timer_value = timer.elapsed_time_us();
        timer.reset_us();
    });

    return 0;
}
