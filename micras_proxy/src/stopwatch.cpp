/**
 * @file
 */

#include <limits>

#include "micras/proxy/stopwatch.hpp"

namespace micras::proxy {
Stopwatch::Stopwatch() {
    this->reset_ms();
}

Stopwatch::Stopwatch(const Config& config) : timer{config.timer} {
    this->reset_us();
}

void Stopwatch::reset_ms() {
    this->counter = hal::Timer::get_counter_ms();
}

void Stopwatch::reset_us() {
    this->counter = this->timer.get_counter_us();
}

uint32_t Stopwatch::elapsed_time_ms() const {
    return hal::Timer::get_counter_ms() - this->counter;
}

uint32_t Stopwatch::elapsed_time_us() const {
    const uint32_t counter = this->timer.get_counter_us();

    if (counter < this->counter) {
        return std::numeric_limits<uint16_t>::max() - this->counter + counter;
    }

    return counter - this->counter;
}

void Stopwatch::sleep_ms(uint32_t time) {
    const uint32_t start = HAL_GetTick();

    while (HAL_GetTick() - start < time) { }
}

void Stopwatch::sleep_us(uint32_t time) const {
    const uint32_t start = this->timer.get_counter_us();

    while (this->timer.get_counter_us() - start < time) { }
}
}  // namespace micras::proxy
