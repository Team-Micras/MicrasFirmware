/**
 * @file test_storage.cpp
 *
 * @brief Test for the Storage class
 *
 * @date 05/2024
 */

#include "test_core.hpp"

using namespace micras;  // NOLINT(google-build-using-namespace)

class TestSerializable : public proxy::ISerializable {
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
        for (uint32_t i = 0; i < this->test_array.size(); i++) {
            this->test_array.at(i) =
                (buffer[i * 4L] << 24) | (buffer[i * 4L + 1L] << 16) | (buffer[i * 4L + 2L] << 8) | buffer[i * 4L + 3L];
        }

        this->test_string = std::string{reinterpret_cast<const char*>(buffer + 40), size - 40U};
    }

    bool operator==(const TestSerializable& other) const {
        return this->test_array == other.test_array and this->test_string == other.test_string;
    }

private:
    std::array<int32_t, 10> test_array{};
    std::string             test_string{};
};

int main(int argc, char* argv[]) {
    TestCore::init(argc, argv);

    proxy::Storage::Config storage_test_config = {.start_page = 0, .number_of_pages = 1};

    proxy::Button  button{button_config};
    proxy::Argb<2> argb{argb_config};
    proxy::Storage storage_0{storage_test_config};

    bool     test_bool_0 = true;
    uint16_t test_uint16_0 = 42;
    float    test_float_0 = 3.14F;

    TestSerializable test_serializable_0{};

    storage_0.create("test_bool", test_bool_0);
    storage_0.create("test_uint16", test_uint16_0);
    storage_0.create("test_float", test_float_0);
    storage_0.create("test_serializable", test_serializable_0);

    storage_0.save();

    proxy::Storage storage_1{storage_test_config};

    bool     test_bool_1 = false;
    uint16_t test_uint16_1 = 0;
    float    test_float_1 = 0.0F;

    TestSerializable test_serializable_1{true};

    storage_1.sync("test_bool", test_bool_1);
    storage_1.sync("test_uint16", test_uint16_1);
    storage_1.sync("test_float", test_float_1);
    storage_1.sync("test_serializable", test_serializable_1);

    TestCore::loop([&test_bool_0, &test_bool_1, &test_uint16_0, &test_uint16_1, &test_float_0, &test_float_1,
                    &test_serializable_0, &test_serializable_1, &button, &argb]() {
        while (button.get_status() == proxy::Button::Status::NO_PRESS) { }

        if (test_bool_0 != test_bool_1) {
            argb.set_color({200, 0, 0});
        } else {
            argb.set_color({0, 200, 0});
        }

        hal::Timer::sleep_ms(500);
        argb.set_color({0, 0, 0});
        hal::Timer::sleep_ms(500);

        if (test_uint16_0 != test_uint16_1) {
            argb.set_color({200, 0, 0});
        } else {
            argb.set_color({0, 200, 0});
        }

        hal::Timer::sleep_ms(500);
        argb.set_color({0, 0, 0});
        hal::Timer::sleep_ms(500);

        if (test_float_0 != test_float_1) {
            argb.set_color({200, 0, 0});
        } else {
            argb.set_color({0, 200, 0});
        }

        hal::Timer::sleep_ms(500);
        argb.set_color({0, 0, 0});
        hal::Timer::sleep_ms(500);

        if (test_serializable_0 != test_serializable_1) {
            argb.set_color({200, 0, 0});
        } else {
            argb.set_color({0, 200, 0});
        }

        hal::Timer::sleep_ms(500);
        argb.set_color({0, 0, 0});
    });

    return 0;
}
