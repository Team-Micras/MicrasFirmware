#ifndef MICRAS_COMM_CUSTOM_SERIAL_VARIABLE_HPP
#define MICRAS_COMM_CUSTOM_SERIAL_VARIABLE_HPP

#include <utility>

#include "micras/comm/vars/serial_variable.hpp"
#include "micras/core/concepts.hpp"
#include "micras/core/utils.hpp"

namespace micras::comm {
/**
 * @brief Class for serializing and deserializing a custom serializable variable.
 *
 * @tparam T Type of the custom serializable variable.
 */
template <core::Serializable T>
class CustomSerialVariable : public ISerialVariable {
public:
    /**
     * @brief Constructor for the CustomSerialVariable class.
     *
     * @param name Name of the variable.
     * @param serializable Pointer to the serializable variable.
     * @param read_only True if the variable is read-only, false otherwise.
     */
    CustomSerialVariable(std::string name, T* serializable, bool read_only) :
        serializable_ptr{serializable}, name{std::move(name)}, read_only{read_only} { }

    /**
     * @brief Get the variable's name.
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
    uint16_t get_size() const override { return 0; }

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
    std::vector<uint8_t> serialize() const override { return serializable_ptr->serialize(); }

    /**
     * @brief Deserialize the variable.
     *
     * @param serial_data Serialized data.
     * @param size Size of the serialized data.
     */
    void deserialize(const uint8_t* serial_data, uint16_t size) override {
        if (read_only) {
            return;
        }
        serializable_ptr->deserialize(serial_data, size);
    }

private:
    /**
     * @brief Pointer to the serializable variable.
     */
    core::ISerializable* serializable_ptr;

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

#endif  // MICRAS_COMM_CUSTOM_SERIAL_VARIABLE_HPP
