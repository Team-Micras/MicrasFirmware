/**
 * @file
 */

#include "test_core.hpp"

using namespace micras;  // NOLINT(google-build-using-namespace)

// NOLINTBEGIN(*-avoid-c-arrays, cppcoreguidelines-avoid-non-const-global-variables)
static volatile float test_reading[4];

static volatile float test_battery_voltage{};

// NOLINTEND(*-avoid-c-arrays, cppcoreguidelines-avoid-non-const-global-variables)

int main(int argc, char* argv[]) {
    TestCore::init(argc, argv);
    hal::Timer            wait_timer{timer_config};
    proxy::Argb<2>        argb{argb_config};
    proxy::WallSensors<4> wall_sensors{wall_sensors_config};
    proxy::Battery        battery{battery_config};

    wall_sensors.turn_on();

    TestCore::loop([&wall_sensors, &argb, &wait_timer, &battery]() {
        wait_timer.reset_us();
        wall_sensors.update();
        battery.update();

        test_battery_voltage = battery.get_voltage();

        for (uint8_t i = 0; i < 4; i++) {
            test_reading[i] = 100.0F * wall_sensors.get_reading(i);
        }

        uint8_t left_blue{};
        uint8_t left_red{};
        uint8_t right_blue{};
        uint8_t right_red{};

        if (wall_sensors.get_observation(0) != micras::core::Observation::FREE_SPACE) {
            left_blue = wall_sensors.get_observation(0) == micras::core::Observation::WALL ? 255 : 127;
        }

        if (wall_sensors.get_observation(3) != micras::core::Observation::FREE_SPACE) {
            right_blue = wall_sensors.get_observation(3) == micras::core::Observation::WALL ? 255 : 127;
        }

        if (wall_sensors.get_observation(1) != micras::core::Observation::FREE_SPACE) {
            left_red = wall_sensors.get_observation(1) == micras::core::Observation::WALL ? 255 : 127;
        }

        if (wall_sensors.get_observation(2) != micras::core::Observation::FREE_SPACE) {
            right_red = wall_sensors.get_observation(2) == micras::core::Observation::WALL ? 255 : 127;
        }

        argb.set_colors({{{right_red, 0, right_blue}, {left_red, 0, left_blue}}});

        while (wait_timer.elapsed_time_us() < 1000) { }
    });

    return 0;
}
