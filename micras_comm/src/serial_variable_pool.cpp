#include "micras/comm/serial_variable_pool.hpp"

// static uint16_t current_id = 0;

// variable map serialization:
// num of primitives
// num of serializables
// id
// name_length
// name
// type_length
// type
// read_only

namespace micras::comm {

uint16_t SerialVariablePool::current_id = 0;

SerialVariablePool::SerialVariablePool() { }

std::vector<uint8_t> SerialVariablePool::serialize_var_map() {
    std::vector<uint8_t> buffer;

    uint16_t count = this->variables.size();
    buffer.emplace_back(count & 0xFF);
    buffer.emplace_back((count >> 8) & 0xFF);

    for (const auto& [id, variable] : this->variables) {
        buffer.push_back(id & 0xFF);
        buffer.push_back((id >> 8) & 0xFF);

        const std::string& name = variable->get_name();
        buffer.push_back(name.size());
        buffer.insert(buffer.end(), name.begin(), name.end());

        const std::string& type = variable->get_type();
        buffer.push_back(type.size());
        buffer.insert(buffer.end(), type.begin(), type.end());

        buffer.push_back(variable->is_read_only());
    }

    return buffer;
}
}  // namespace micras::comm
