/**
 * @file
 */

#include <cstdio>
#include <cstring>

#include "micras/core/variable_pool.hpp"
#include "test_host.hpp"

using namespace micras::core;

struct Blob : ISerializable {
    std::vector<uint8_t> data{1, 2, 3};

    std::vector<uint8_t> serialize() const override { return data; }

    void deserialize(const uint8_t* p, uint16_t n) override { data.assign(p, p + n); }
};

enum class Profile : uint8_t {
    A,
    B
};

int main() {
    TVariablePool<8> pool;
    float            speed = 1.5F;
    const float      k = 2.0F;
    Profile          profile = Profile::A;
    Blob             blob;

    auto id_speed = pool.add("speed/", "linear", speed, {.stream = true, .write = true});
    auto id_k = pool.add("model/", "kv", k, {.stream = true, .write = true, .idle = true, .persist = true});
    auto id_prof = pool.add("", "run_profile", profile, {.write = true, .idle = true, .persist = true});
    auto id_blob = pool.add("", "maze", blob, {.persist = true});

    CHECK(id_speed == 0 && id_k == 1 && id_prof == 2 && id_blob == 3);
    CHECK(pool.all().size() == 4);
    CHECK(pool.at(id_k).access.write == false);
    CHECK(pool.at(id_prof).type == TypeCode::U8);
    CHECK(pool.at(id_blob).type == TypeCode::BLOB);
    CHECK(pool.at(id_blob).access.stream == false);

    CHECK(pool.find("speed/linear").value() == 0);
    CHECK(pool.find("model/kv").value() == 1);
    CHECK(pool.find("run_profile").value() == 2);
    CHECK(!pool.find("speed/linea").has_value());
    CHECK(!pool.find("speed/linearr").has_value());

    uint8_t buf[8];
    CHECK(pool.read(id_speed, buf) == 4);
    float out;
    std::memcpy(&out, buf, 4);
    CHECK(out == 1.5F);
    CHECK(pool.read(id_blob, buf) == 0);

    float nv = 9.0F;
    CHECK(pool.write(id_speed, {reinterpret_cast<uint8_t*>(&nv), 4}, false) == VariablePool::WriteStatus::OK);
    CHECK(speed == 9.0F);
    CHECK(pool.write(id_k, {reinterpret_cast<uint8_t*>(&nv), 4}, true) == VariablePool::WriteStatus::READ_ONLY);
    uint8_t one = 1;
    CHECK(pool.write(id_prof, {&one, 1}, false) == VariablePool::WriteStatus::NEEDS_IDLE);
    CHECK(pool.write(id_prof, {&one, 1}, true) == VariablePool::WriteStatus::OK);
    CHECK(profile == Profile::B);
    CHECK(pool.write(id_speed, {&one, 1}, true) == VariablePool::WriteStatus::WRONG_SIZE);
    CHECK(pool.write(99, {&one, 1}, true) == VariablePool::WriteStatus::NO_SUCH_ID);

    uint32_t         h0 = pool.schema_hash();
    TVariablePool<8> other;
    other.add("speed/", "linear", speed, {.stream = true, .write = true});
    other.add("model/", "kv", k, {.stream = true, .write = true, .idle = true, .persist = true});
    other.add("", "run_profile", profile, {.write = true, .idle = true, .persist = true});
    other.add("", "maze", blob, {.persist = true});
    CHECK(other.schema_hash() == h0);
    other.add("", "extra", speed, {});
    CHECK(other.schema_hash() != h0);

    TVariablePool<1> tiny;
    tiny.add("", "a", speed, {});
    CHECK(tiny.add("", "b", speed, {}) == VariablePool::invalid_id);
    CHECK(tiny.all().size() == 1);

    std::printf("pool ok, sizeof(Variable)=%zu hash=%08X\n", sizeof(Variable), h0);
}
