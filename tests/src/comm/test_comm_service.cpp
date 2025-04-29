#include "test_core.hpp"
#include "micras/comm/communication_service.hpp"
#include "micras/comm/serial_variable_pool.hpp"
#include "micras/comm/logger.hpp"

using namespace micras;  // NOLINT(google-build-using-namespace)

class TestSerializable : public core::ISerializable {
public:
    explicit TestSerializable() {
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
            this->test_array.at(i) =
                (buffer[i * 4L] << 24) | (buffer[i * 4L + 1L] << 16) | (buffer[i * 4L + 2L] << 8) | buffer[i * 4L + 3L];
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

int main(int argc, char* argv[]) {
    TestCore::init(argc, argv);

    proxy::Bluetooth bluetooth{bluetooth_config};

    comm::SerialVariablePool   pool{};
    comm::Logger               logger{true};
    comm::CommunicationService comm_service{pool, logger};

    comm_service.register_communication_functions(
        [&bluetooth](const std::vector<uint8_t>& data) { bluetooth.send_data(data); },
        [&bluetooth]() { return bluetooth.get_data(); }
    );

    uint32_t write_only_var = 0;
    uint32_t read_only_var = 0;

    TestSerializable test_serializable{};

    pool.add_read_only("read_only_var", read_only_var);
    pool.add_write_only("write_only_var", write_only_var);
    pool.add_read_only("test_serializable", test_serializable);

    TestCore::loop([&comm_service, &bluetooth, &write_only_var, &read_only_var]() {
        comm_service.update();
        bluetooth.update();

        read_only_var = write_only_var;
    });

    return 0;
}
