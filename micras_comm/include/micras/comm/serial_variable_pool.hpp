#ifndef MICRAS_COMM_SERIAL_VARIABLE_POOL_HPP
#define MICRAS_COMM_SERIAL_VARIABLE_POOL_HPP

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "micras/comm/vars/custom_serial_variable.hpp"
#include "micras/comm/vars/primitive_serial_variable.hpp"
#include "micras/comm/vars/serial_variable.hpp"
#include "micras/core/serializable.hpp"
#include "micras/core/utils.hpp"

namespace micras::comm {
class SerialVariablePool {
public:
    /**
     * @brief Enum for the type of serial variable.
     */
    enum class VarType : uint8_t {
        MONITORING = 0,
        BIDIRECTIONAL = 1,
    };

    /**
     * @brief Construct a new Serial Variable Pool object.
     */
    explicit SerialVariablePool();

    /**
     * @brief Add a primitive variable to the pool.
     *
     * @tparam Type of the primitive variable.
     * @param name Name of the variable.
     * @param data Reference to the variable.
     * @param type Type of the variable (default is VarType::MONITORING).
     */
    template <core::Fundamental T>
    void add_variable(const std::string& name, T& data, VarType type = VarType::MONITORING) {
        const uint16_t var_id = current_id++;
        this->variables[var_id] =
            std::make_unique<PrimitiveSerialVariable<T>>(name, &data, type == VarType::MONITORING);
    }

    /**
     * @brief Add a serializable variable to the pool.
     *
     * @tparam Type of the serializable variable.
     * @param name Name of the variable.
     * @param data Reference to the variable.
     * @param type Type of the variable (default is VarType::MONITORING).
     */
    template <core::Serializable T>
    void add_variable(const std::string& name, T& data, VarType type = VarType::MONITORING) {
        const uint16_t var_id = current_id++;
        this->variables[var_id] = std::make_unique<CustomSerialVariable<T>>(name, &data, type == VarType::MONITORING);
    }

    /**
     * @brief Write data to a variable in the pool.
     *
     * @param id ID of the variable.
     * @param data Data to write.
     */
    void update_variable(uint16_t id, std::vector<uint8_t> data);

    /**
     * @brief Get the number of variables in the pool.
     *
     * @return Number of variables in the pool.
     */
    uint16_t size() const;

    /**
     * @brief Iterate over all variables in the pool and call the provided callback function.
     *
     * @tparam Func Type of the callback function.
     * @param callback Callback function to call for each variable.
     */
    template <typename Func>
    void for_each(Func callback) {
        for (const auto& [id, variable] : this->variables) {
            callback(id, *variable);
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
