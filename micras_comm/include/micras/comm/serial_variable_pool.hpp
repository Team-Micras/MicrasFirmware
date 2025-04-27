#ifndef MICRAS_COMM_SERIAL_VARIABLE_POOL_HPP
#define MICRAS_COMM_SERIAL_VARIABLE_POOL_HPP

#include <cstdint>
#include <string>
#include <vector>
#include <memory>

#include <unordered_map>

#include "micras/core/serializable.hpp"
#include "micras/core/utils.hpp"

#include "micras/comm/serial_variable.hpp"
#include "micras/comm/primitive_serial_variable.hpp"
#include "micras/comm/custom_serial_variable.hpp"

namespace micras::comm {
class SerialVariablePool {
public:
    explicit SerialVariablePool();

    template <core::Fundamental T>
    void add_read_only(const std::string& name, T& data) {
        uint16_t var_id = current_id++;
        this->variables[var_id] = std::make_unique<PrimitiveSerialVariable<T>>(name, &data, true);
    }

    template <core::Fundamental T>
    void add_write_only(const std::string& name, T& data) {
        uint16_t var_id = current_id++;
        this->variables[var_id] = std::make_unique<PrimitiveSerialVariable<T>>(name, &data, false);
    }

    template <core::Serializable T>
    void add_read_only(const std::string& name, T& data) {
        uint16_t var_id = current_id++;
        this->variables[var_id] = std::make_unique<CustomSerialVariable<T>>(name, &data, true);
    }

    template <core::Serializable T>
    void add_write_only(const std::string& name, T& data) {
        uint16_t var_id = current_id++;
        this->variables[var_id] = std::make_unique<CustomSerialVariable<T>>(name, &data, false);
    }

    std::vector<uint8_t> serialize_var_map();

private:
    static uint16_t current_id;

    std::unordered_map<uint16_t, std::unique_ptr<ISerialVariable>> variables;

    static std::vector<uint8_t> serialize_var_map();
};
}  // namespace micras::comm

#endif  // MICRAS_COMM_SERIAL_VARIABLE_POOL_HPP
