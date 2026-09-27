/**
 * @file
 */

#include <cstdint>

#include "micras/hal/timer.hpp"
#include "micras/proxy/tick.hpp"

namespace micras::proxy {
Tick::Tick(const Config& config) :
    period{hal::Timer::to_cycles(config.period_us)}, last_tick{hal::Timer::get_counter()} { }

uint32_t Tick::wait() {
    while (hal::Timer::get_counter() - this->last_tick < this->period) { }

    const uint32_t ticks = (hal::Timer::get_counter() - this->last_tick) / this->period;
    this->last_tick += ticks * this->period;

    return ticks;
}

uint32_t Tick::elapsed_time_us() const {
    return hal::Timer::to_microseconds(hal::Timer::get_counter() - this->last_tick);
}
}  // namespace micras::proxy
