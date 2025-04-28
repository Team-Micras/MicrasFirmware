#ifndef MICRAS_COMM_SERIAL_VARIABLE_POOL_HPP
#define MICRAS_COMM_SERIAL_VARIABLE_POOL_HPP

#include <cstdint>
#include <string>
#include <vector>
#include <memory>

#include <unordered_map>

#include "micras/core/serializable.hpp"
#include "micras/core/utils.hpp"

#include "micras/comm/vars/serial_variable.hpp"
#include "micras/comm/vars/primitive_serial_variable.hpp"
#include "micras/comm/vars/custom_serial_variable.hpp"

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

    void write(uint16_t id, std::vector<uint8_t> data);

    template<typename Func>
    void for_each_read_only_variable(Func callback) {
        for (const auto& [id, variable] : this->variables) {
            if (variable->is_read_only()) {
                callback(id, *variable);
            }
        }
    }

    std::vector<uint8_t> serialize_var_map();

private:
    static uint16_t current_id;

    std::unordered_map<uint16_t, std::unique_ptr<ISerialVariable>> variables;
};
}  // namespace micras::comm

#endif  // MICRAS_COMM_SERIAL_VARIABLE_POOL_HPP
