/**
 * @file
 */

#include "test_core.hpp"

using namespace micras;  // NOLINT(google-build-using-namespace)

int main(int argc, char* argv[]) {
    TestCore::init(argc, argv);
    proxy::Led led{led_config};

    TestCore::loop([&led]() {
        led.toggle();

        proxy::Stopwatch::sleep_ms(500);
    });

    return 0;
}
