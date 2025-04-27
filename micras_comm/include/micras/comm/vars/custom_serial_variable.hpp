#ifndef MICRAS_COMM_CUSTOM_SERIAL_VARIABLE_HPP
#define MICRAS_COMM_CUSTOM_SERIAL_VARIABLE_HPP

#include <type_traits>

#include "micras/core/concepts.hpp"
#include "micras/core/utils.hpp"
#include "micras/comm/vars/serial_variable.hpp"

namespace micras::comm {

template <core::Serializable T>
class CustomSerialVariable : public ISerialVariable {
private:
    core::ISerializable* serializable_ptr;
    std::string          name;
    bool                 read_only;

public:
    CustomSerialVariable(const std::string& name, T* serializable, bool read_only) :
        serializable_ptr(serializable), name(name), read_only(read_only) { }

    std::string get_name() const override { return name; }

    std::string get_type() const override { return core::type_name<T>(); }

    uint16_t get_size() const override { return 0; }

    bool is_read_only() const override { return read_only; }

    std::vector<uint8_t> serialize() const override { return serializable_ptr->serialize(); }

    void deserialize(const uint8_t* serial_data, uint16_t size) override {
        if (read_only)
            return;
        serializable_ptr->deserialize(serial_data, size);
    }
};
}  // namespace micras::comm

#endif  // MICRAS_COMM_CUSTOM_SERIAL_VARIABLE_HPP
