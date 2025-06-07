/**
 * @file
 */

#ifndef MICRAS_CORE_SERIAL_VARIABLES_PRIMITIVE_HPP
#define MICRAS_CORE_SERIAL_VARIABLES_PRIMITIVE_HPP

#include <cstring>

#include "micras/core/serial/variables/base.hpp"
#include "micras/core/concepts.hpp"

namespace micras::core {
/**
 * @brief Interface for variables to serialized and deserialized for communication purposes.
 */
class PrimitiveSerialVariable : public SerialVariable {
public:
    /**
     * @brief Constructor for the PrimitiveSerialVariable class.
     *
     * @param name Name of the variable.
     * @param data Pointer to the variable.
     * @param read_only True if the variable is read-only, false otherwise.
     */
    PrimitiveSerialVariable(std::string name, void* data, uint16_t size, bool read_only, std::string type) :
        SerialVariable{name, read_only, type, size}, data_ptr{data} { }

    PrimitiveSerialVariable(std::string name, void* data, uint16_t size, bool read_only, uint16_t buffer_address) :
        SerialVariable{name, read_only, buffer_address, size}, data_ptr{data} { }

    /**
     * @brief Serialize the variable.
     *
     * @return Serialized data.
     */
    std::vector<uint8_t> serialize() const override {
        std::vector<uint8_t> result(this->get_size());
        std::memcpy(result.data(), this->data_ptr, this->get_size());
        return result;
    }

    /**
     * @brief Deserialize the variable.
     *
     * @param serial_data Pointer to the serialized data.
     * @param size Size of the serialized data.
     */
    void deserialize(const uint8_t* serial_data, uint16_t size) override {
        std::memcpy(this->data_ptr, serial_data, size);
    }

private:
    /**
     * @brief
     *
     */
    void* data_ptr;
};
}  // namespace micras::core

#endif  // MICRAS_CORE_SERIAL_VARIABLES_PRIMITIVE_HPP
