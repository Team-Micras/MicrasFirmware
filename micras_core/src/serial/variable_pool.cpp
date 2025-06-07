/**
 * @file
 */

#include "micras/core/serial/variable_pool.hpp"
#include "micras/core/serial/variables/custom.hpp"
#include "micras/core/serial/variables/primitive.hpp"

namespace micras::core {

void SerialVariablePool::update_variable(uint16_t id, std::vector<uint8_t> data) {
    auto it = this->variables.find(id);
    if (it != this->variables.end()) {
        it->second->deserialize(data.data(), data.size());
    }
}

std::vector<uint8_t> SerialVariablePool::serialize_map() {
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

        const auto& extra_info = variable->get_extra_info();
        buffer.insert(buffer.end(), extra_info.begin(), extra_info.end());

        // @todo: remover read_only
        buffer.push_back(static_cast<uint8_t>(variable->is_read_only()));

        // bool is_primitive = (typeid(*variable) == typeid(core::PrimitiveSerialVariable));

        // if (is_primitive) {
        //     buffer.push_back(0x01);
        //     buffer.push_back(variable->get_size());
        //     buffer.push_back(variable->get_size() >> 8);
        // } else {
        //     buffer.push_back(0x00);
        // }
    }

    return buffer;
}

void SerialVariablePool::deserialize_map(const std::vector<uint8_t>& data) {
    size_t         offset = 0;
    const uint16_t count = data[offset] | (data[offset + 1] << 8);
    offset += 2;

    for (uint16_t i = 0; i < count; ++i) {
        uint16_t id = data[offset] | (data[offset + 1] << 8);
        offset += 2;

        uint8_t           name_length = data[offset++];
        const std::string var_name(data.begin() + offset, data.begin() + offset + name_length);
        offset += name_length;

        uint16_t buffer_address = data[offset] | (data[offset + 1] << 8);
        offset += 2;

        bool read_only = static_cast<bool>(data[offset++]);

        bool is_primitive = static_cast<bool>(data[offset++]);

        if (is_primitive) {
            uint16_t size = data[offset] | (data[offset + 1] << 8);
            offset += 2;
            this->variables[id] =
                std::make_unique<PrimitiveSerialVariable>(var_name, nullptr, size, read_only, buffer_address);
        } else {
            this->variables[id] = std::make_unique<CustomSerialVariable>(var_name, nullptr, read_only, buffer_address);
        }
    }
}
}  // namespace micras::core
