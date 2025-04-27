#ifndef MICRAS_COMM_SERIAL_VARIABLE_HPP
#define MICRAS_COMM_SERIAL_VARIABLE_HPP

//todo nao gostei do nome da classe

#include <string>

#include "micras/core/serializable.hpp"

namespace micras::comm {

class ISerialVariable : public core::ISerializable {
public:
    virtual ~ISerialVariable() = default;

    virtual std::string get_name() const = 0;
    virtual std::string get_type() const = 0;
    virtual uint16_t get_size() const = 0;
    virtual bool is_read_only() const = 0;
};

}

#endif  // MICRAS_COMM_SERIAL_VARIABLE_HPP
