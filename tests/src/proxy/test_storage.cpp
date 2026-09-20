/**
 * @file
 */

#include <array>
#include <bit>
#include <cstdint>
#include <string>
#include <vector>
#include "micras/core/serializable.hpp"
#include "micras/core/variable_pool.hpp"
#include "micras/proxy/button.hpp"
#include "micras/proxy/stopwatch.hpp"
#include "micras/proxy/storage.hpp"
#include "target.hpp"
#include "test_core.hpp"

using namespace micras;  // NOLINT(google-build-using-namespace)

static constexpr uint32_t time_interval{500};

namespace {
class TestSerializable : public core::ISerializable {
public:
    explicit TestSerializable(bool empty = false) {
        if (empty) {
            return;
        }

        for (uint32_t i = 0; i < this->test_array.size(); i++) {
            this->test_array.at(i) = i;
        }

        this->test_string = "Hello, World!";
    }

    std::vector<uint8_t> serialize() const override {
        std::vector<uint8_t> buffer;

        for (const auto& value : test_array) {
            buffer.emplace_back(value >> 24);
            buffer.emplace_back(value >> 16);
            buffer.emplace_back(value >> 8);
            buffer.emplace_back(value);
        }

        for (const auto& value : test_string) {
            buffer.emplace_back(value);
        }

        return buffer;
    }

    void deserialize(const uint8_t* buffer, uint16_t size) override {
        // NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        for (uint32_t i = 0; i < this->test_array.size(); i++) {
            this->test_array.at(i) = (static_cast<uint32_t>(buffer[i * 4L]) << 24) |
                                     (static_cast<uint32_t>(buffer[i * 4L + 1L]) << 16) |
                                     (static_cast<uint32_t>(buffer[i * 4L + 2L]) << 8) | buffer[i * 4L + 3L];
        }

        this->test_string = std::string{std::bit_cast<const char*>(buffer + 40), size - 40U};
        // NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    }

    bool operator==(const TestSerializable& other) const {
        return this->test_array == other.test_array and this->test_string == other.test_string;
    }

private:
    std::array<uint32_t, 10> test_array{};
    std::string              test_string;
};
}  // namespace

int main(int argc, char* argv[]) {
    TestCore::init(argc, argv);

    const proxy::Storage::Config storage_test_config = {.start_sector = 0, .number_of_sectors = 1};

    proxy::Button  button{button_config};
    proxy::Argb    argb{argb_config};
    proxy::Storage storage_0{storage_test_config};

    bool    test_bool_0 = true;
    int16_t test_int16_0 = 42;
    float   test_float_0 = 3.14F;

    TestSerializable test_serializable_0{};

    core::TVariablePool<8> pool_0;
    pool_0.add("test/", "bool", test_bool_0, {.persist = true});
    pool_0.add("test/", "int16", test_int16_0, {.persist = true});
    pool_0.add("test/", "float", test_float_0, {.persist = true});
    pool_0.add("test/", "serializable", test_serializable_0, {.persist = true});

    storage_0.save(pool_0);

    proxy::Storage storage_1{storage_test_config};

    bool    test_bool_1 = false;
    int16_t test_int16_1 = 0;
    float   test_float_1 = 0.0F;

    TestSerializable test_serializable_1{true};

    core::TVariablePool<8> pool_1;
    pool_1.add("test/", "bool", test_bool_1, {.persist = true});
    pool_1.add("test/", "int16", test_int16_1, {.persist = true});
    pool_1.add("test/", "float", test_float_1, {.persist = true});
    pool_1.add("test/", "serializable", test_serializable_1, {.persist = true});

    const bool restored = storage_1.restore(pool_1) == 4 and pool_1.schema_hash() == pool_0.schema_hash();

    TestCore::loop([&test_bool_0, &test_bool_1, &test_int16_0, &test_int16_1, &test_float_0, &test_float_1,
                    &test_serializable_0, &test_serializable_1, &restored, &button, &argb]() {
        while (button.get_status() == proxy::Button::Status::NO_PRESS) {
            button.update();
        }

        if (test_bool_0 != test_bool_1) {
            argb.set_color(proxy::Argb::Colors::red);
        } else {
            argb.set_color(proxy::Argb::Colors::green);
        }

        proxy::Stopwatch::sleep_ms(time_interval);
        argb.turn_off();
        proxy::Stopwatch::sleep_ms(time_interval);

        if (test_int16_0 != test_int16_1) {
            argb.set_color(proxy::Argb::Colors::red);
        } else {
            argb.set_color(proxy::Argb::Colors::green);
        }

        proxy::Stopwatch::sleep_ms(time_interval);
        argb.turn_off();
        proxy::Stopwatch::sleep_ms(time_interval);

        if (test_float_0 != test_float_1) {
            argb.set_color(proxy::Argb::Colors::red);
        } else {
            argb.set_color(proxy::Argb::Colors::green);
        }

        proxy::Stopwatch::sleep_ms(time_interval);
        argb.turn_off();
        proxy::Stopwatch::sleep_ms(time_interval);

        if (test_serializable_0 != test_serializable_1) {
            argb.set_color(proxy::Argb::Colors::red);
        } else {
            argb.set_color(proxy::Argb::Colors::green);
        }

        proxy::Stopwatch::sleep_ms(time_interval);
        argb.turn_off();
        proxy::Stopwatch::sleep_ms(time_interval);

        if (not restored) {
            argb.set_color(proxy::Argb::Colors::red);
        } else {
            argb.set_color(proxy::Argb::Colors::green);
        }

        proxy::Stopwatch::sleep_ms(time_interval);
        argb.turn_off();
    });

    return 0;
}
