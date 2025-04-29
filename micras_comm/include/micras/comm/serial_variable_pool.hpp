#ifndef MICRAS_COMM_SERIAL_VARIABLE_POOL_HPP
#define MICRAS_COMM_SERIAL_VARIABLE_POOL_HPP

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "micras/core/utils.hpp"
#include "micras/core/serializable.hpp"
#include "micras/comm/vars/custom_serial_variable.hpp"
#include "micras/comm/vars/primitive_serial_variable.hpp"
#include "micras/comm/vars/serial_variable.hpp"

namespace micras::comm {
class SerialVariablePool {
public:
    /**
     * @brief Construct a new Serial Variable Pool object.
     */
    explicit SerialVariablePool();

    /**
     * @brief Add a read-only primitive variable to the pool.
     *
     * @tparam T Type of the primitive variable.
     * @param name Name of the variable.
     * @param data Reference to the variable.
     */
    template <core::Fundamental T>
    void add_read_only(const std::string& name, T& data) {
        uint16_t var_id = current_id++;
        this->variables[var_id] = std::make_unique<PrimitiveSerialVariable<T>>(name, &data, true);
    }

    /**
     * @brief Add a write-only primitive variable to the pool.
     *
     * @tparam T Type of the primitive variable.
     * @param name Name of the variable.
     * @param data Reference to the variable.
     */
    template <core::Fundamental T>
    void add_write_only(const std::string& name, T& data) {
        uint16_t var_id = current_id++;
        this->variables[var_id] = std::make_unique<PrimitiveSerialVariable<T>>(name, &data, false);
    }

    /**
     * @brief Add a read-only serializable variable to the pool.
     *
     * @tparam T Type of the serializable variable.
     * @param name Name of the variable.
     * @param data Reference to the variable.
     */
    template <core::Serializable T>
    void add_read_only(const std::string& name, T& data) {
        uint16_t var_id = current_id++;
        this->variables[var_id] = std::make_unique<CustomSerialVariable<T>>(name, &data, true);
    }

    /**
     * @brief Add a write-only serializable variable to the pool.
     *
     * @tparam T Type of the serializable variable.
     * @param name Name of the variable.
     * @param data Reference to the variable.
     */
    template <core::Serializable T>
    void add_write_only(const std::string& name, T& data) {
        uint16_t var_id = current_id++;
        this->variables[var_id] = std::make_unique<CustomSerialVariable<T>>(name, &data, false);
    }

    /**
     * @brief Write data to a variable in the pool.
     *
     * @param id ID of the variable.
     * @param data Data to write.
     */
    void write(uint16_t id, std::vector<uint8_t> data);

    /**
     * @brief Iterate over all read-only variables in the pool and call the provided callback function.
     *
     * @tparam Func Type of the callback function.
     * @param callback Callback function to call for each read-only variable.
     */
    template <typename Func>
    void for_each_read_only_variable(Func callback) {
        for (const auto& [id, variable] : this->variables) {
            if (variable->is_read_only()) {
                callback(id, *variable);
            }
        }
    }

    /**
     * @brief Serialize the variable map to a byte array.
     *
     * @return std::vector<uint8_t> Serialized variable map.
     *
     * @note The variable map is serialized in the following format:
     * | Offset | Description            | Size (bytes)       |
     * |--------|------------------------|--------------------|
     * | 0      | Num of variables       | 2                  |
     * | 2      | ID                     | 2                  |
     * | 4      | Name Length            | 1                  |
     * | 5      | Name                   | Name Length        |
     * | ...    | Type Name Length       | 1                  |
     * | ...    | Type                   | Type Name Length   |
     * | ...    | Read-Only Flag         | 1                  |
     */
    std::vector<uint8_t> serialize_var_map();

private:
    /**
     * @brief Current ID for the variables.
     */
    static uint16_t current_id;

    /**
     * @brief Map of variable IDs to their corresponding serializable variables.
     */
    std::unordered_map<uint16_t, std::unique_ptr<ISerialVariable>> variables;
};
}  // namespace micras::comm

#endif  // MICRAS_COMM_SERIAL_VARIABLE_POOL_HPP
