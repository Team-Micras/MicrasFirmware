/**
 * @file
 */

#ifndef MICRAS_CORE_SERIAL_VARIABLE_POOL_HPP
#define MICRAS_CORE_SERIAL_VARIABLE_POOL_HPP

#include <cstdint>
#include <memory>
#include <unordered_map>
#include <vector>

#include "micras/core/concepts.hpp"
#include "micras/core/serial/variables/custom.hpp"
#include "micras/core/serial/variables/primitive.hpp"
#include "micras/core/utils.hpp"

namespace micras::core {
class SerialVariablePool {
public:
    enum class VarType : uint8_t {
        MONITORING = 0,
        BIDIRECTIONAL = 1,
    };

    /**
     * @brief Construct a new Serial Variable Pool object.
     */
    explicit SerialVariablePool() = default;

    /**
     * @brief Add a read-only primitive variable to the pool.
     *
     * @tparam T Type of the primitive variable.
     * @param name Name of the variable.
     * @param data Reference to the variable.
     */
    template <core::Fundamental T>
    void add_variable(const std::string& name, T& data, uint16_t size, VarType type = VarType::BIDIRECTIONAL) {
        const uint16_t var_id = this->id_counter++;
        this->variables[var_id] = std::make_unique<PrimitiveSerialVariable>(name, &data, size, type);
    }

    template <core::Serializable T>
    void add_variable(const std::string& name, T& data, VarType type = VarType::BIDIRECTIONAL) {
        const uint16_t var_id = this->id_counter++;
        this->variables[var_id] = std::make_unique<CustomSerialVariable>(name, &data, type);
    }

    /**
     * @brief Deserialize data into the variable with the given ID.
     *
     * @param id ID of the variable.
     * @param data Data to write.
     */
    void update_variable(uint16_t id, std::vector<uint8_t> data);

    /**
     * @brief Iterate over all read-only variables in the pool and call the provided callback function.
     *
     * @tparam Func Type of the callback function.
     * @param callback Callback function to call for each read-only variable.
     */
    template <typename Func>  //@todo tirar isso
    void for_each_read_only_variable(Func callback) {
        for (const auto& [id, variable] : this->variables) {
            if (variable->is_read_only()) {
                callback(id, *variable);
            }
        }
    }

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
    std::vector<uint8_t> serialize_map();

    void deserialize_map(const std::vector<uint8_t>& data);

private:
    /**
     * @brief Current ID for the variables.
     */
    uint16_t id_counter{};

    /**
     * @brief Map of variable IDs to their corresponding serializable variables.
     */
    std::unordered_map<uint16_t, std::unique_ptr<SerialVariable>> variables;
};
}  // namespace micras::core

#endif  // MICRAS_CORE_SERIAL_VARIABLE_POOL_HPP
