#ifndef MICRAS_COMM_SERIAL_VARIABLE_HPP
#define MICRAS_COMM_SERIAL_VARIABLE_HPP

#include <string>

#include "micras/core/serializable.hpp"

namespace micras::comm {
/**
 * @brief Interface for variables to serialized and deserialized for communication purposes.
 */
class ISerialVariable : public core::ISerializable {
public:
    /**
     * @brief Virtual destructor for the ISerialVariable class.
     */
    virtual ~ISerialVariable() = default;

    /**
     * @brief Get the name of the variable.
     *
     * @return Name of the variable.
     */
    virtual std::string get_name() const = 0;

    /**
     * @brief Get the type of the variable.
     *
     * @return String representation of the variable type.
     */
    virtual std::string get_type() const = 0;

    /**
     * @brief Get the size of the variable.
     *
     * @return Size of the variable.
     */
    virtual uint16_t    get_size() const = 0;

    /**
     * @brief Check if the variable is read-only.
     *
     * @return True if the variable is read-only, false otherwise.
     */
    virtual bool        is_read_only() const = 0;
};

}  // namespace micras::comm

#endif  // MICRAS_COMM_SERIAL_VARIABLE_HPP
