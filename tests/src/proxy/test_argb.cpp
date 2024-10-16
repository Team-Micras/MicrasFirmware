/**
 * @file test_argb.cpp
 *
 * @brief Test for the Addressable RGB class
 *
 * @date 05/2024
 */

#include "test_core.hpp"

using namespace micras;  // NOLINT(google-build-using-namespace)

int main(int argc, char* argv[]) {
    TestCore::init(argc, argv);

    proxy::Argb<2>        argb{argb_config};
    proxy::Argb<2>::Color color{255, 0, 0};

    TestCore::loop([&argb, &color]() {
        for (uint8_t i = 1; i > 0; i++) {
            color.blue = i;
            argb.set_color(color);
            hal::Timer::sleep_ms(2);
        }

        for (uint8_t i = 254; i < 255; i--) {
            color.red = i;
            argb.set_color(color);
            hal::Timer::sleep_ms(2);
        }

        for (uint8_t i = 1; i > 0; i++) {
            color.green = i;
            argb.set_color(color);
            hal::Timer::sleep_ms(2);
        }

        for (uint8_t i = 254; i < 255; i--) {
            color.blue = i;
            argb.set_color(color);
            hal::Timer::sleep_ms(2);
        }

        for (uint8_t i = 1; i > 0; i++) {
            color.red = i;
            argb.set_color(color);
            hal::Timer::sleep_ms(2);
        }

        for (uint8_t i = 254; i < 255; i--) {
            color.green = i;
            argb.set_color(color);
            hal::Timer::sleep_ms(2);
        }
    });

    return 0;
}
