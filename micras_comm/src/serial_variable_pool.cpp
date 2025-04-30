#include "micras/comm/serial_variable_pool.hpp"

namespace micras::comm {

uint16_t SerialVariablePool::current_id = 0;

SerialVariablePool::SerialVariablePool() = default;

void SerialVariablePool::write(uint16_t id, std::vector<uint8_t> data) {
    auto it = this->variables.find(id);
    if (it != this->variables.end()) {
        it->second->deserialize(data.data(), data.size());
    }
}

std::vector<uint8_t> SerialVariablePool::serialize_var_map() {
    std::vector<uint8_t> buffer;

    const uint16_t count = this->variables.size();
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

        buffer.push_back(static_cast<uint8_t>(variable->is_read_only()));
    }

    return buffer;
}
}  // namespace micras::comm
