/**
 * @file test_buzzer.cpp
 *
 * @brief Test for the Buzzer class
 *
 * @date 05/2024
 */

#include "hal/timer.hpp"
#include "proxy/buzzer.hpp"
#include "target.hpp"
#include "test_core.hpp"

int main() {
    test_core_init();
    proxy::Buzzer buzzer{buzzer_config};

    while (true) {
        buzzer.play(987, 133);
        hal::Timer::sleep_ms(333);
        buzzer.play(987, 133);
        hal::Timer::sleep_ms(333);
        buzzer.play(987, 133);
        hal::Timer::sleep_ms(333);
        buzzer.play(987, 133);
        hal::Timer::sleep_ms(333);
        buzzer.play(987, 133);
        hal::Timer::sleep_ms(250);
        buzzer.play(1046, 133);
        hal::Timer::sleep_ms(250);
        buzzer.play(1318, 133);
        hal::Timer::sleep_ms(167);
        buzzer.play(987, 133);
        hal::Timer::sleep_ms(333);
        buzzer.play(987, 133);
        hal::Timer::sleep_ms(333);
        buzzer.play(987, 133);
        hal::Timer::sleep_ms(250);
        buzzer.play(1046, 133);
        hal::Timer::sleep_ms(250);
        buzzer.play(1318, 133);
        hal::Timer::sleep_ms(167);
        buzzer.play(987, 133);
        hal::Timer::sleep_ms(333);
        buzzer.play(987, 133);
        hal::Timer::sleep_ms(333);
        buzzer.play(987, 133);
        hal::Timer::sleep_ms(250);
        buzzer.play(1046, 133);
        hal::Timer::sleep_ms(250);
        buzzer.play(1318, 133);
        hal::Timer::sleep_ms(167);
        buzzer.play(1479, 133);
        hal::Timer::sleep_ms(333);
        buzzer.play(1479, 133);
        hal::Timer::sleep_ms(333);
        buzzer.play(1760, 133);
        hal::Timer::sleep_ms(250);
        buzzer.play(1567, 133);
        hal::Timer::sleep_ms(250);
        buzzer.play(1479, 133);
        hal::Timer::sleep_ms(167);
        buzzer.play(1318, 133);
        hal::Timer::sleep_ms(333);
        buzzer.play(1318, 133);
        hal::Timer::sleep_ms(333);
        buzzer.play(1760, 133);
        hal::Timer::sleep_ms(250);
        buzzer.play(1567, 133);
        hal::Timer::sleep_ms(250);
        buzzer.play(1479, 133);
        hal::Timer::sleep_ms(167);
        buzzer.play(1318, 133);
        hal::Timer::sleep_ms(333);
        buzzer.play(1318, 133);
        hal::Timer::sleep_ms(333);
        buzzer.play(987, 133);
        hal::Timer::sleep_ms(250);
        buzzer.play(1046, 133);
        hal::Timer::sleep_ms(250);
        buzzer.play(1318, 133);
        hal::Timer::sleep_ms(167);
        buzzer.play(987, 133);
        hal::Timer::sleep_ms(333);
        buzzer.play(987, 133);
        hal::Timer::sleep_ms(333);
        buzzer.play(987, 133);
        hal::Timer::sleep_ms(250);
        buzzer.play(1046, 133);
        hal::Timer::sleep_ms(250);
        buzzer.play(1318, 133);
        hal::Timer::sleep_ms(167);
        buzzer.play(987, 133);
        hal::Timer::sleep_ms(333);
        buzzer.play(987, 133);
        hal::Timer::sleep_ms(5000);
    }

    return 0;
}
