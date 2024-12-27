/**
 * @file
 */

#include "test_core.hpp"

using namespace micras;  // NOLINT(google-build-using-namespace)

static volatile uint8_t test_dip_switch_value{};  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

int main(int argc, char* argv[]) {
    TestCore::init(argc, argv);
    proxy::DipSwitch<4>   dip_switch{dip_switch_config};
    proxy::Argb<2>        argb{argb_config};
    proxy::Argb<2>::Color color{};

    TestCore::loop([&dip_switch, &argb, &color]() {
        color = {0, 0, 0};
        test_dip_switch_value = dip_switch.get_switches_value();

        if (dip_switch.get_switch_state(0)) {
            color.red = 127;
        }

        if (dip_switch.get_switch_state(1)) {
            color.green = 127;
        }

        if (dip_switch.get_switch_state(2)) {
            color.blue = 127;
        }

        if (dip_switch.get_switch_state(3)) {
            color.red *= 2;
            color.green *= 2;
            color.blue *= 2;
        }

        argb.set_color(color);
        hal::Timer::sleep_ms(2);
    });

    return 0;
}
