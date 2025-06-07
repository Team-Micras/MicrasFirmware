/**
 * @file
 */

#ifndef MICRAS_CORE_SERIAL_VARIABLES_BASE_HPP
#define MICRAS_CORE_SERIAL_VARIABLES_BASE_HPP

#include <string>

#include "micras/core/serial/serializable.hpp"
#include "micras/core/utils.hpp"

namespace micras::core {
/**
 * @brief Interface for variables to serialized and deserialized for communication purposes.
 */
class SerialVariable : public core::ISerializable {
public:
    /**
     * @brief Constructor for the SerialVariable class.
     *
     * @param name Name of the variable.
     * @param read_only True if the variable is read-only, false otherwise.
     * @param size Size of the variable.
     */
    SerialVariable(std::string name, bool read_only, std::string type, uint16_t size = 0) :
        name{std::move(name)}, read_only{read_only}, size{size} {
        this->extra_info.emplace_back(type.size());
        this->extra_info.insert(this->extra_info.end(), type.begin(), type.end());
    }

    SerialVariable(std::string name, bool read_only, uint16_t buffer_address, uint16_t size) :
        name{std::move(name)}, read_only{read_only}, size{size} {
        this->extra_info.emplace_back(buffer_address);
        this->extra_info.emplace_back(buffer_address >> 8);
    }

    /**
     * @brief Virtual destructor for the SerialVariable class.
     */
    ~SerialVariable() override = default;

    /**
     * @brief Get the variable's name.
     *
     * @return Name of the variable.
     */
    std::string get_name() const { return this->name; }

    /**
     * @brief Check if the variable is read-only.
     *
     * @return True if the variable is read-only, false otherwise.
     */
    bool is_read_only() const { return this->read_only; }

    /**
     * @brief Get the size of the variable.
     *
     * @return Size of the variable.
     */
    uint16_t get_size() const { return this->size; }

    /**
     * @brief Get the extra information of the variable.
     *
     * @return Extra information of the variable.
     */
    std::vector<uint8_t> get_extra_info() const { return this->extra_info; }

private:
    /**
     * @brief Special member functions declared as default.
     */
    ///@{
    SerialVariable(const SerialVariable&) = default;
    SerialVariable(SerialVariable&&) = default;
    SerialVariable& operator=(const SerialVariable&) = default;
    SerialVariable& operator=(SerialVariable&&) = default;
    ///@}

    /**
     * @brief Name of the variable.
     */
    std::string name;

    bool read_only;

    uint16_t size;

    std::vector<uint8_t> extra_info;
};
}  // namespace micras::core

#endif  // MICRAS_CORE_SERIAL_VARIABLES_BASE_HPP
