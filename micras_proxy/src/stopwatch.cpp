/**
 * @file
 */

#include <cstdint>

#include "micras/hal/timer.hpp"
#include "micras/proxy/stopwatch.hpp"

namespace micras::proxy {
Stopwatch::Stopwatch() {
    this->reset_ms();
    this->reset_us();
}

void Stopwatch::reset_ms() {
    this->counter_ms = hal::Timer::get_counter_ms();
}

void Stopwatch::reset_us() {
    this->counter_cycles = hal::Timer::get_counter();
}

uint32_t Stopwatch::elapsed_time_ms() const {
    return hal::Timer::get_counter_ms() - this->counter_ms;
}

uint32_t Stopwatch::elapsed_time_us() const {
    return hal::Timer::to_microseconds(hal::Timer::get_counter() - this->counter_cycles);
}

void Stopwatch::sleep_ms(uint32_t time) {
    const uint32_t start = hal::Timer::get_counter_ms();

    while (hal::Timer::get_counter_ms() - start < time) { }
}

void Stopwatch::sleep_us(uint32_t time) {
    const uint32_t start = hal::Timer::get_counter();
    const uint32_t cycles = hal::Timer::to_cycles(time);

    while (hal::Timer::get_counter() - start < cycles) { }
}
}  // namespace micras::proxy
