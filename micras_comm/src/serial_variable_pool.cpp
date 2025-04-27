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

void SerialVariablePool::add_read_only(const std::string& name, const proxy::ISerializable& data) {
    this->add_variable(name, true, data);
}

void SerialVariablePool::add_write_only(const std::string& name, const proxy::ISerializable& data) {
    this->add_variable(name, false, data);
}

void SerialVariablePool::add_variable(const std::string& name, bool is_read_only, const proxy::ISerializable& data) {
    uint16_t var_id = current_id++;
    this->serializables[var_id].ram_pointer = &data;
    this->serializables[var_id].name = name;
    this->serializables[var_id].type = type_name<proxy::ISerializable>();
    this->serializables[var_id].size = sizeof(data);
    this->serializables[var_id].is_read_only = is_read_only;
}

std::vector<uint8_t> SerialVariablePool::serialize_var_map() {
    std::vector<uint8_t> buffer;

    buffer.emplace_back(this->primitives.size());
    buffer.emplace_back(this->primitives.size() >> 8);

    buffer.emplace_back(this->serializables.size());
    buffer.emplace_back(this->serializables.size() >> 8);

    auto serialized_primitives = serialize_var_map<PrimitiveVariable>(this->primitives);
    buffer.insert(buffer.begin(), serialized_primitives.begin(), serialized_primitives.end());

    auto serialized_serializables = serialize_var_map<SerializableVariable>(this->serializables);
    buffer.insert(buffer.begin(), serialized_serializables.begin(), serialized_serializables.end());

    return buffer;
}

template <typename T>
std::vector<uint8_t> SerialVariablePool::serialize_var_map(const std::unordered_map<uint16_t, T>& variable_map) {
    std::vector<uint8_t> buffer;

    for (const auto& [id, variable] : variable_map) {
        buffer.emplace_back(id);
        buffer.emplace_back(id >> 8);

        buffer.emplace_back(variable.name.size());
        buffer.insert(buffer.end(), variable.name.begin(), variable.name.end());

        buffer.emplace_back(variable.type.size());
        buffer.insert(buffer.end(), variable.type.begin(), variable.type.end());

        buffer.emplace_back(variable.is_read_only);
    }

    return buffer;
}

template std::vector<uint8_t>
    SerialVariablePool::serialize_var_map(const std::unordered_map<uint16_t, PrimitiveVariable>& variable_map);
template std::vector<uint8_t>
    SerialVariablePool::serialize_var_map(const std::unordered_map<uint16_t, SerializableVariable>& variable_map);
}  // namespace micras::comm
