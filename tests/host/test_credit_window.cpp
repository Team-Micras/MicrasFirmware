/**
 * @file
 */

#include <cstdint>
#include <cstdio>

#include "micras/comm/credit_window.hpp"
#include "micras/comm/protocol.hpp"
#include "test_host.hpp"

using namespace micras::comm;

int main() {
    // --- the window holds exactly its size in flight ---
    {
        CreditWindow window;
        CHECK(window.available() == credit_window);
        CHECK(window.allows(credit_window));
        CHECK(not window.allows(credit_window + 1));

        window.charge(200);
        CHECK(window.outstanding() == 200);
        CHECK(window.available() == credit_window - 200);
        CHECK(window.allows(56));
        CHECK(not window.allows(57));

        window.acknowledge(150);
        CHECK(window.outstanding() == 50);
        CHECK(window.allows(206));
    }

    // --- a late total is ignored ---
    {
        CreditWindow window;
        window.charge(100);
        window.acknowledge(80);
        window.acknowledge(40);
        CHECK(window.outstanding() == 20);
        window.acknowledge(80);
        CHECK(window.outstanding() == 20);
    }

    // --- a total ahead of what was sent is ignored, and a partial one after it still counts ---
    {
        CreditWindow window;
        window.charge(100);
        window.acknowledge(101);
        CHECK(window.outstanding() == 100);
        window.acknowledge(5000);
        CHECK(window.outstanding() == 100);
        window.acknowledge(60);
        CHECK(window.outstanding() == 40);
        window.acknowledge(100);
        CHECK(window.outstanding() == 0);
        CHECK(window.sent_total() == 100);
    }

    // --- the total sent is what an application resynchronizes to ---
    {
        CreditWindow window;
        window.charge(90);
        window.charge(70);
        window.acknowledge(90);
        CHECK(window.outstanding() == 70);
        window.acknowledge(window.sent_total());
        CHECK(window.outstanding() == 0);
    }

    // --- a lost credit is recovered by the next one ---
    {
        CreditWindow window;
        window.charge(120);
        window.charge(120);
        CHECK(not window.allows(20));
        window.acknowledge(240);
        CHECK(window.outstanding() == 0);
    }

    // --- both totals wrap around at 32 bits without the window noticing ---
    {
        CreditWindow window;
        uint32_t     consumed = 0;
        uint64_t     sent = 0;
        uint32_t     wraps = 0;

        while (sent < (uint64_t{1} << 32U) + 10 * credit_window) {
            CHECK(window.allows(200));
            window.charge(200);
            sent += 200;
            CHECK(window.outstanding() == 200);

            const uint32_t previous = consumed;
            consumed += 200;
            wraps += consumed < previous ? 1 : 0;

            window.acknowledge(previous);
            CHECK(window.outstanding() == 200);
            window.acknowledge(consumed);
            CHECK(window.outstanding() == 0);
            CHECK(window.available() == credit_window);
        }

        CHECK(wraps == 1);
        window.charge(100);
        window.acknowledge(consumed - 1000);
        CHECK(window.outstanding() == 100);
        window.acknowledge(consumed + 60);
        CHECK(window.outstanding() == 40);
    }

    // --- a new session starts from nothing ---
    {
        CreditWindow window;
        window.charge(250);
        window.reset();
        CHECK(window.outstanding() == 0);
        window.acknowledge(10);
        CHECK(window.outstanding() == 0);
    }

    std::puts("credit window ok");
}
