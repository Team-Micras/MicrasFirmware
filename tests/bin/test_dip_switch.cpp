/**
 * @file test_dip_switch.cpp
 *
 * @brief Test for the DipSwitch class
 *
 * @date 05/2024
 */

#include "test_core.hpp"

int main() {
    test_core_init();
    proxy::DipSwitch<4>   dip_switch{dip_switch_config};
    proxy::Argb<2>        argb{argb_config};
    proxy::Argb<2>::Color color{};
    uint8_t               dip_switch_value{};

    while (true) {
        color = {0, 0, 0};
        dip_switch_value = dip_switch.get_switches_value();

        if (dip_switch.get_switch_state(0)) {
            color.red = 128;
        }

        if (dip_switch.get_switch_state(1)) {
            color.green = 128;
        }

        if (dip_switch.get_switch_state(2)) {
            color.blue = 128;
        }

        if (dip_switch.get_switch_state(3)) {
            color.red *= 2;
            color.green *= 2;
            color.blue *= 2;
        }

        argb.set_color(color);
    }

    return 0;
}
