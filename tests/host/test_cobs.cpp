/**
 * @file
 */

#include <cstdio>
#include <numeric>
#include <random>
#include <vector>

#include "micras/core/cobs.hpp"
#include "test_host.hpp"

using namespace micras::core;
using V = std::vector<uint8_t>;

static V enc(const V& in) {
    V out(cobs_encoded_size(in.size()));
    out.resize(cobs_encode(in, out));
    return out;
}

static V dec(const V& in) {
    V out(in.size() + 1);
    out.resize(cobs_decode(in, out));
    return out;
}

static void vec(const V& raw, const V& expected) {
    V got = enc(raw);
    if (got != expected) {
        std::printf("encode mismatch for %zu bytes: got %zu expected %zu\n", raw.size(), got.size(), expected.size());

        for (auto b : got) {
            std::printf("%02X ", b);
        }

        std::puts("");

        for (auto b : expected) {
            std::printf("%02X ", b);
        }

        std::puts("");
        CHECK(false);
    }
    CHECK(dec(got) == raw);
}

static V range(int a, int b) {
    V v;
    for (int i = a; i <= b; i++)
        v.push_back(uint8_t(i));
    return v;
}

int main() {
    vec({}, {0x01});
    vec({0x00}, {0x01, 0x01});
    vec({0x00, 0x00}, {0x01, 0x01, 0x01});
    vec({0x00, 0x11, 0x00}, {0x01, 0x02, 0x11, 0x01});
    vec({0x11, 0x22, 0x00, 0x33}, {0x03, 0x11, 0x22, 0x02, 0x33});
    vec({0x11, 0x22, 0x33, 0x44}, {0x05, 0x11, 0x22, 0x33, 0x44});
    vec({0x11, 0x00, 0x00, 0x00}, {0x02, 0x11, 0x01, 0x01, 0x01});

    {
        V r = range(1, 254);
        V e{0xFF};
        for (auto b : r)
            e.push_back(b);
        vec(r, e);
    }
    {
        V r = range(0, 254);
        V e{0x01, 0xFF};
        for (int i = 1; i <= 254; i++)
            e.push_back(i);
        vec(r, e);
    }
    {
        V r = range(1, 255);
        V e{0xFF};
        for (int i = 1; i <= 254; i++)
            e.push_back(i);
        e.push_back(0x02);
        e.push_back(0xFF);
        vec(r, e);
    }
    {
        V r = range(2, 255);
        r.push_back(0x00);
        V e{0xFF};
        for (int i = 2; i <= 255; i++)
            e.push_back(i);
        e.push_back(0x01);
        e.push_back(0x01);
        vec(r, e);
    }
    {
        V r = range(3, 255);
        r.push_back(0x00);
        r.push_back(0x01);
        V e{0xFE};
        for (int i = 3; i <= 255; i++)
            e.push_back(i);
        e.push_back(0x02);
        e.push_back(0x01);
        vec(r, e);
    }

    // every encoded frame is free of the delimiter, and round trips
    std::mt19937 rng{7};
    for (int trial = 0; trial < 200000; trial++) {
        V raw(rng() % 600);
        for (auto& b : raw)
            b = uint8_t(rng() % (trial % 3 == 0 ? 3 : 256));
        V e = enc(raw);
        CHECK(e.size() <= cobs_encoded_size(raw.size()));
        for (auto b : e)
            CHECK(b != 0);
        CHECK(dec(e) == raw);
    }

    // malformed frames are rejected rather than half decoded
    V scratch(8);
    CHECK(cobs_decode(V{0x00}, scratch) == 0);
    CHECK(cobs_decode(V{0x05, 0x01}, scratch) == 0);
    {
        V out(2);
        CHECK(cobs_decode(V{0x06, 1, 2, 3, 4, 5}, out) == 0);
    }

    std::puts("cobs ok");
}
