#ifndef MICRAS_COMM_PRIMITIVE_SERIAL_VARIABLE_HPP
#define MICRAS_COMM_PRIMITIVE_SERIAL_VARIABLE_HPP

#include <type_traits>
#include "serial_variable.hpp"

namespace micras::comm {

//todo concept no core
template <typename T>
concept Fundamental = std::is_fundamental_v<T>;

template <Fundamental T>
class PrimitiveSerialVariable : public ISerialVariable {

public:
    PrimitiveSerialVariable(const std::string& name, T* value, bool read_only)
        : value_ptr(value), name(name), read_only(read_only) {}

    std::string get_name() const override { return name; }
    std::string get_type() const override { return type_name<T>(); }
    uint16_t get_size() const override { return sizeof(T); }
    bool is_read_only() const override { return read_only; }

    std::vector<uint8_t> serialize() const override {
        std::vector<uint8_t> result(sizeof(T));
        std::memcpy(result.data(), value_ptr, sizeof(T));
        return result;
    }

    void deserialize(const uint8_t* serial_data, uint16_t size) override {
        if (read_only || size != sizeof(T)) return;
        std::memcpy(value_ptr, serial_data, sizeof(T));
    }

private:
    T* value_ptr;
    std::string name;
    bool read_only;
};
}

#endif  // MICRAS_COMM_PRIMITIVE_SERIAL_VARIABLE_HPP
