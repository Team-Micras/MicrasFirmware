/**
 * @file
 */

#ifndef MICRAS_CORE_SERIAL_VARIABLES_CUSTOM_HPP
#define MICRAS_CORE_SERIAL_VARIABLES_CUSTOM_HPP

#include <cstring>

#include "micras/core/serial/variables/base.hpp"
#include "micras/core/concepts.hpp"

namespace micras::core {
/**
 * @brief Interface for variables to serialized and deserialized for communication purposes.
 */
class CustomSerialVariable : public SerialVariable {
public:
    /**
     * @brief Constructor for the CustomSerialVariable class.
     *
     * @param name Name of the variable.
     * @param data Pointer to the variable.
     * @param read_only True if the variable is read-only, false otherwise.
     */
    CustomSerialVariable(std::string name, ISerializable* data, bool read_only, std::string type) :
        SerialVariable{name, read_only, type}, data_ptr{data} { }

    CustomSerialVariable(
        std::string name, ISerializable* data, bool read_only, uint16_t buffer_address, uint16_t size
    ) :
        SerialVariable{name, read_only, buffer_address, size}, data_ptr{data} { }

    /**
     * @brief Serialize the variable.
     *
     * @return Serialized data.
     */
    std::vector<uint8_t> serialize() const override { return data_ptr->serialize(); }

    /**
     * @brief Deserialize the variable.
     *
     * @param serial_data Pointer to the serialized data.
     * @param size Size of the serialized data.
     */
    void deserialize(const uint8_t* serial_data, uint16_t size) override { data_ptr->deserialize(serial_data, size); }

private:
    /**
     * @brief Pointer to the variable.
     */
    ISerializable* data_ptr;
};
}  // namespace micras::core

#endif  // MICRAS_CORE_SERIAL_VARIABLES_CUSTOM_HPP
