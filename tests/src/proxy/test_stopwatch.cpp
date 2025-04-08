/**
 * @file
 */

#include "test_core.hpp"

using namespace micras;  // NOLINT(google-build-using-namespace)

static volatile uint32_t test_stopwatch_value{};  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

int main(int argc, char* argv[]) {
    TestCore::init(argc, argv);
    proxy::Stopwatch stopwatch{stopwatch_config};

    TestCore::loop([&stopwatch]() {
        test_stopwatch_value += stopwatch.elapsed_time_us();
        stopwatch.reset_us();
        proxy::Stopwatch::sleep_ms(2);
    });

    return 0;
}
