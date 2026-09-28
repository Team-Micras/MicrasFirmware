/**
 * @file
 */

#include <cstdint>
#include <cstdio>

#include "micras/hal/timer.hpp"
#include "micras/proxy/microsecond_clock.hpp"
#include "test_host.hpp"

using micras::hal::Timer;
using micras::proxy::MicrosecondClock;

int main() {
    // --- starts at zero, from any value of the cycle counter ---
    {
        Timer::counter = 123'456'789U;
        MicrosecondClock clock;
        CHECK(clock.now_us() == 0);
    }

    // --- goes past the wrap of the cycle counter, 2^32 cycles, about 7.8 s at 550 MHz ---
    {
        Timer::counter = 0xFFFF'0000U;
        MicrosecondClock clock;
        uint32_t         previous = 0;

        for (uint32_t i = 1; i <= 20'000; i++) {
            Timer::counter += 550U * 1000U;  // one iteration of the loop, 1 ms
            const uint32_t now = clock.now_us();
            CHECK(now == previous + 1000U);
            previous = now;
        }

        CHECK(previous == 20'000'000U);  // 20 s, where the cycle counter wrapped twice
    }

    // --- carries the cycles that do not make a whole microsecond, so it does not drift ---
    {
        Timer::counter = 0;
        MicrosecondClock clock;

        for (uint32_t i = 0; i < 1'000'000; i++) {
            Timer::counter += 549U;  // just under a microsecond at a time
        }
        CHECK(clock.now_us() == 549U * 1'000'000U / 550U);

        for (uint32_t i = 0; i < 1'000; i++) {
            Timer::counter += 549U;
            clock.now_us();
        }
        CHECK(clock.now_us() == 549U * 1'001'000U / 550U);
    }

    // --- wraps only at 2^32 us, about 71.6 min ---
    {
        Timer::counter = 0;
        Timer::cycles_per_microsecond = 1;
        MicrosecondClock clock;

        Timer::counter = 0xFFFF'FFF0U;
        CHECK(clock.now_us() == 0xFFFF'FFF0U);
        Timer::counter += 0x20U;
        CHECK(clock.now_us() == 0x10U);
        Timer::cycles_per_microsecond = 550;
    }

    std::printf("microsecond clock ok\n");
    return 0;
}
