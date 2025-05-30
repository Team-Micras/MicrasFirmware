/**
 * @file
 */

#include "test_core.hpp"

using namespace micras;  // NOLINT(google-build-using-namespace)

// NOLINTBEGIN(*-avoid-c-arrays, cppcoreguidelines-avoid-non-const-global-variables)
static volatile float test_reading[4];
static volatile float test_adc_reading[4];

// NOLINTEND(*-avoid-c-arrays, cppcoreguidelines-avoid-non-const-global-variables)

int main(int argc, char* argv[]) {
    TestCore::init(argc, argv);
    proxy::WallSensors wall_sensors{wall_sensors_config};
    proxy::Argb        argb{argb_config};

    wall_sensors.turn_on();

    TestCore::loop([&wall_sensors, &argb]() {
        wall_sensors.update();

        for (uint8_t i = 0; i < 4; i++) {
            test_reading[i] = wall_sensors.get_reading(i);
            test_adc_reading[i] = wall_sensors.get_adc_reading(i);
        }

        for (uint8_t i = 0; i < 2; i++) {
            proxy::Argb::Color color{};
            color.red = wall_sensors.get_wall(3 - 3 * i) ? 255 : 0;
            color.blue = wall_sensors.get_wall(2 - i) ? 255 : 0;

            argb.set_color(color, i);
        }

        proxy::Stopwatch::sleep_ms(2);
    });

    return 0;
}
