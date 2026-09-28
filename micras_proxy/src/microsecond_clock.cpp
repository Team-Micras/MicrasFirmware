/**
 * @file
 */

#include <cstdint>

#include "micras/hal/timer.hpp"
#include "micras/proxy/microsecond_clock.hpp"

namespace micras::proxy {
MicrosecondClock::MicrosecondClock() : last_counter{hal::Timer::get_counter()} { }

uint32_t MicrosecondClock::now_us() {
    const uint32_t counter = hal::Timer::get_counter();
    const uint32_t cycles = counter - this->last_counter + this->remainder_cycles;
    const uint32_t elapsed_us = hal::Timer::to_microseconds(cycles);

    this->last_counter = counter;
    this->remainder_cycles = cycles - hal::Timer::to_cycles(elapsed_us);
    this->time_us += elapsed_us;

    return this->time_us;
}
}  // namespace micras::proxy
