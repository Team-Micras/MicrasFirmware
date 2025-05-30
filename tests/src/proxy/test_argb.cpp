/**
 * @file
 */

#include "test_core.hpp"

using namespace micras;  // NOLINT(google-build-using-namespace)

int main(int argc, char* argv[]) {
    TestCore::init(argc, argv);

    proxy::Argb        argb{argb_config};
    proxy::Argb::Color color{255, 0, 0};

    TestCore::loop([&argb, &color]() {
        for (uint8_t i = 1; i > 0; i++) {
            color.blue = i;
            argb.set_color(color);
            proxy::Stopwatch::sleep_ms(2);
        }

        for (uint8_t i = 254; i < 255; i--) {
            color.red = i;
            argb.set_color(color);
            proxy::Stopwatch::sleep_ms(2);
        }

        for (uint8_t i = 1; i > 0; i++) {
            color.green = i;
            argb.set_color(color);
            proxy::Stopwatch::sleep_ms(2);
        }

        for (uint8_t i = 254; i < 255; i--) {
            color.blue = i;
            argb.set_color(color);
            proxy::Stopwatch::sleep_ms(2);
        }

        for (uint8_t i = 1; i > 0; i++) {
            color.red = i;
            argb.set_color(color);
            proxy::Stopwatch::sleep_ms(2);
        }

        for (uint8_t i = 254; i < 255; i--) {
            color.green = i;
            argb.set_color(color);
            proxy::Stopwatch::sleep_ms(2);
        }
    });

    return 0;
}
