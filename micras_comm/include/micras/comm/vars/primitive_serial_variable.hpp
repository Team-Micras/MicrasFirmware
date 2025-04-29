#ifndef MICRAS_COMM_PRIMITIVE_SERIAL_VARIABLE_HPP
#define MICRAS_COMM_PRIMITIVE_SERIAL_VARIABLE_HPP

#include <type_traits>
#include <cstring>

#include "micras/core/concepts.hpp"
#include "micras/core/utils.hpp"
#include "micras/comm/vars/serial_variable.hpp"

namespace micras::comm {
/**
 * @brief Class for serializing and deserializing primitive variables.
 *
 * @tparam T Type of the primitive variable.
 */
template <core::Fundamental T>
class PrimitiveSerialVariable : public ISerialVariable {
public:
    /**
     * @brief Constructor for the PrimitiveSerialVariable class.
     *
     * @param name Name of the variable.
     * @param value Pointer to the variable.
     * @param read_only True if the variable is read-only, false otherwise.
     */
    PrimitiveSerialVariable(const std::string& name, T* value, bool read_only) :
        value_ptr(value), name(name), read_only(read_only) { }

    /**
     * @brief Get the varaible's name.
     *
     * @return std::string name of the variable.
     */
    std::string get_name() const override { return name; }

    /**
     * @brief Get the variable's type.
     *
     * @return std::string type of the variable.
     */
    std::string get_type() const override { return std::string(core::type_name<T>()); }

    /**
     * @brief Get the variable's size.
     *
     * @return uint16_t size of the variable.
     */
    uint16_t get_size() const override { return sizeof(T); }

    /**
     * @brief Check if the variable is read-only.
     *
     * @return bool true if the variable is read-only, false otherwise.
     */
    bool is_read_only() const override { return read_only; }

    /**
     * @brief Serialize the variable.
     *
     * @return std::vector<uint8_t> serialized data.
     */
    std::vector<uint8_t> serialize() const override {
        std::vector<uint8_t> result(sizeof(T));
        std::memcpy(result.data(), value_ptr, sizeof(T));
        return result;
    }

    /**
     * @brief Deserialize the variable.
     *
     * @param serial_data Pointer to the serialized data.
     * @param size Size of the serialized data.
     */
    void deserialize(const uint8_t* serial_data, uint16_t size) override {
        if (read_only || size != sizeof(T))
            return;
        std::memcpy(value_ptr, serial_data, sizeof(T));
    }

private:
    /**
     * @brief Pointer to the variable.
     */
    T* value_ptr;

    /**
     * @brief Name of the variable.
     */
    std::string name;

    /**
     * @brief Read-only flag.
     */
    bool read_only;
};
}  // namespace micras::comm

#endif  // MICRAS_COMM_PRIMITIVE_SERIAL_VARIABLE_HPP
