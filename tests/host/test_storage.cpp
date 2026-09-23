/**
 * @file
 */

#include <cstdio>

#include "micras/hal/flash.hpp"
#include "micras/proxy/storage.hpp"
#include "test_host.hpp"

using namespace micras;

struct Blob : core::ISerializable {
    std::vector<uint8_t> data;

    std::vector<uint8_t> serialize() const override { return data; }

    void deserialize(const uint8_t* p, uint16_t n) override { data.assign(p, p + n); }
};

static const proxy::Storage::Config cfg{.start_sector = 0, .number_of_sectors = 1};

int main() {
    // --- round trip ---
    {
        core::TVariablePool<8> pool;
        float                  kv = 3.25F;
        uint8_t                profile = 7;
        bool                   flag = true;
        float                  transient = 1.0F;
        Blob                   maze;
        maze.data = {9, 8, 7, 6, 5};
        pool.add("model/", "kv", kv, {.persist = true});
        pool.add("", "run_profile", profile, {.persist = true});
        pool.add("", "flag", flag, {.persist = true});
        pool.add("", "transient", transient, {.stream = true});
        pool.add("", "maze", maze, {.persist = true});

        proxy::Storage s{cfg};
        CHECK(!s.is_valid());
        CHECK(s.restore(pool) == 0);
        CHECK(s.save(pool));
        CHECK(s.is_valid());
    }
    {
        core::TVariablePool<8> pool;
        float                  kv = 0.0F;
        uint8_t                profile = 0;
        bool                   flag = false;
        float                  transient = 42.0F;
        Blob                   maze;
        pool.add("model/", "kv", kv, {.persist = true});
        pool.add("", "run_profile", profile, {.persist = true});
        pool.add("", "flag", flag, {.persist = true});
        pool.add("", "transient", transient, {.stream = true});
        pool.add("", "maze", maze, {.persist = true});

        proxy::Storage s{cfg};
        CHECK(s.is_valid());
        CHECK(s.restore(pool) == 4);
        CHECK(kv == 3.25F && profile == 7 && flag == true);
        CHECK((maze.data == std::vector<uint8_t>{9, 8, 7, 6, 5}));
        CHECK(transient == 42.0F);
    }
    // --- a renamed / reordered pool still finds what it knows ---
    {
        core::TVariablePool<8> pool;
        uint8_t                profile = 0;
        float                  kv = 0.0F;
        float                  fresh = 1.0F;
        pool.add("", "fresh", fresh, {.persist = true});
        pool.add("", "run_profile", profile, {.persist = true});
        pool.add("model/", "kv", kv, {.persist = true});
        proxy::Storage s{cfg};
        CHECK(s.restore(pool) == 2);
        CHECK(profile == 7 && kv == 3.25F && fresh == 1.0F);
    }
    // --- a type change is refused, not misread ---
    {
        core::TVariablePool<8> pool;
        uint32_t               kv = 0;
        pool.add("model/", "kv", kv, {.persist = true});
        proxy::Storage s{cfg};
        CHECK(s.restore(pool) == 0);
        CHECK(kv == 0);
    }
    // --- a torn write leaves the previous image, not a half one ---
    {
        core::TVariablePool<8> pool;
        float                  kv = 99.0F;
        pool.add("model/", "kv", kv, {.persist = true});
        proxy::Storage s{cfg};
        hal::Flash::write_budget = 0;
        CHECK(!s.save(pool));
        hal::Flash::write_budget = UINT32_MAX;
        proxy::Storage after{cfg};
        CHECK(!after.is_valid());
        CHECK(after.restore(pool) == 0);
    }
    // --- writing the body but dying before the header is the same as never saving ---
    {
        hal::Flash::erase_sectors(0, 1);
        core::TVariablePool<8> pool;
        float                  kv = 1.0F;
        pool.add("model/", "kv", kv, {.persist = true});
        proxy::Storage s{cfg};
        hal::Flash::write_budget = 1;  // one flash word: the body goes in, the header does not
        CHECK(!s.save(pool));
        hal::Flash::write_budget = UINT32_MAX;
        proxy::Storage after{cfg};
        CHECK(!after.is_valid());
    }
    std::puts("storage ok");
}
