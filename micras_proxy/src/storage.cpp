/**
 * @file
 */

#include <algorithm>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <unordered_map>
#include <vector>

#include "micras/core/serializable.hpp"
#include "micras/hal/flash.hpp"
#include "micras/proxy/storage.hpp"

namespace micras::proxy {
/**
 * @brief Read a 16 bit little endian value from a buffer.
 *
 * @param buffer Buffer to read from.
 * @param address Address of the first byte of the value.
 * @return Value read from the buffer.
 */
static uint16_t read_uint16(std::span<const uint8_t> buffer, uint16_t address) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-avoid-unchecked-container-access)
    return static_cast<uint16_t>(buffer[address] | buffer[address + 1U] << 8);
}

/**
 * @brief Append a 16 bit little endian value to a buffer.
 *
 * @param buffer Buffer to append to.
 * @param value Value to append.
 */
static void append_uint16(std::vector<uint8_t>& buffer, uint16_t value) {
    buffer.emplace_back(value);
    buffer.emplace_back(value >> 8);
}

/**
 * @brief Check if the data of every variable of a map fits inside the buffer.
 *
 * @tparam T Type of the variables.
 * @param variables Map of variables to check.
 * @param buffer_size Number of bytes of the buffer.
 * @return True if every variable is inside the buffer, false otherwise.
 */
template <typename T>
static bool validate_var_map(const std::unordered_map<std::string, T>& variables, std::size_t buffer_size) {
    return std::ranges::all_of(variables, [buffer_size](const auto& variable) {
        return variable.second.buffer_address + static_cast<std::size_t>(variable.second.size) <= buffer_size;
    });
}

Storage::Storage(const Config& config) :
    start_sector{config.start_sector}, number_of_sectors{config.number_of_sectors} {
    const std::span<const uint8_t> header = hal::Flash::read(this->start_sector, 0, header_size);

    if (header.size() < header_size or read_uint16(header, 0) != start_symbol) {
        return;
    }

    const uint16_t total_size = read_uint16(header, 2);
    const uint16_t num_primitives = read_uint16(header, 4);
    const uint16_t num_serializables = read_uint16(header, 6);

    const std::span<const uint8_t> payload = hal::Flash::read(this->start_sector, header_size, 4UL * total_size);

    if (payload.size() < 4UL * total_size) {
        return;
    }

    this->buffer.assign(payload.begin(), payload.end());

    if (not deserialize_var_map<PrimitiveVariable>(this->buffer, num_primitives, this->primitives) or
        not deserialize_var_map<SerializableVariable>(this->buffer, num_serializables, this->serializables) or
        not validate_var_map(this->primitives, this->buffer.size()) or
        not validate_var_map(this->serializables, this->buffer.size())) {
        this->primitives.clear();
        this->serializables.clear();
        this->buffer.clear();
        return;
    }

    this->valid = true;
}

bool Storage::is_valid() const {
    return this->valid;
}

void Storage::create(const std::string& name, const core::ISerializable& data) {
    this->serializables[name].ram_pointer = &data;
}

void Storage::sync(const std::string& name, core::ISerializable& data) {
    if (this->serializables.contains(name) and this->serializables.at(name).ram_pointer == nullptr) {
        const auto& serializable = this->serializables.at(name);
        data.deserialize(&this->buffer.at(serializable.buffer_address), serializable.size);
    }

    this->create(name, data);
}

bool Storage::save() {
    this->buffer.clear();

    for (auto it = this->primitives.begin(); it != this->primitives.end();) {
        auto& [name, variable] = *it;

        if (variable.ram_pointer == nullptr) {
            it = this->primitives.erase(it);
            continue;
        }

        const auto* aux = std::bit_cast<const uint8_t*>(variable.ram_pointer);
        variable.buffer_address = buffer.size();

        // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        this->buffer.insert(this->buffer.end(), aux, aux + variable.size);
        it++;
    }

    for (auto it = this->serializables.begin(); it != this->serializables.end();) {
        auto& [name, variable] = *it;

        if (variable.ram_pointer == nullptr) {
            it = this->serializables.erase(it);
            continue;
        }

        std::vector<uint8_t> aux = variable.ram_pointer->serialize();
        variable.buffer_address = this->buffer.size();
        variable.size = aux.size();
        this->buffer.insert(this->buffer.end(), aux.begin(), aux.end());
        it++;
    }

    auto serialized_serializables = serialize_var_map<SerializableVariable>(this->serializables);
    this->buffer.insert(this->buffer.begin(), serialized_serializables.begin(), serialized_serializables.end());

    auto serialized_primitives = serialize_var_map<PrimitiveVariable>(this->primitives);
    this->buffer.insert(this->buffer.begin(), serialized_primitives.begin(), serialized_primitives.end());

    this->buffer.insert(this->buffer.end(), (4 - (this->buffer.size() % 4)) % 4, 0);
    const uint16_t total_size = this->buffer.size() / 4;

    std::vector<uint8_t> header;
    header.reserve(header_size);
    append_uint16(header, start_symbol);
    append_uint16(header, total_size);
    append_uint16(header, this->primitives.size());
    append_uint16(header, this->serializables.size());

    this->buffer.insert(this->buffer.begin(), header.begin(), header.end());

    if (this->buffer.size() > this->number_of_sectors * hal::Flash::sector_size) {
        return false;
    }

    if (hal::Flash::erase_sectors(this->start_sector, this->number_of_sectors) != hal::Flash::Status::OK) {
        return false;
    }

    if (hal::Flash::write(this->start_sector, 0, this->buffer) != hal::Flash::Status::OK) {
        return false;
    }

    this->valid = true;
    return true;
}

template <typename T>
std::vector<uint8_t> Storage::serialize_var_map(const std::unordered_map<std::string, T>& variables) {
    std::vector<uint8_t> buffer;

    for (const auto& [name, variable] : variables) {
        buffer.emplace_back(name.size());
        buffer.insert(buffer.end(), name.begin(), name.end());

        buffer.emplace_back(variable.buffer_address);
        buffer.emplace_back(variable.buffer_address >> 8);

        buffer.emplace_back(variable.size);
        buffer.emplace_back(variable.size >> 8);
    }

    return buffer;
}

template <typename T>
bool Storage::deserialize_var_map(
    std::vector<uint8_t>& buffer, uint16_t num_vars, std::unordered_map<std::string, T>& variables
) {
    uint16_t current_addr = 0;

    for (uint16_t decoded_vars = 0; decoded_vars < num_vars; decoded_vars++) {
        if (current_addr >= buffer.size()) {
            return false;
        }

        const uint8_t var_name_len = buffer.at(current_addr);

        if (current_addr + var_name_len + 5UL > buffer.size()) {
            return false;
        }

        const std::string var_name(buffer.begin() + current_addr + 1, buffer.begin() + current_addr + 1 + var_name_len);
        current_addr += var_name_len + 1;

        variables[var_name].buffer_address = read_uint16(buffer, current_addr);
        current_addr += 2;

        variables.at(var_name).size = read_uint16(buffer, current_addr);
        current_addr += 2;
    }

    buffer.erase(buffer.begin(), buffer.begin() + current_addr);
    return true;
}

// Explicit instantiation of template functions
template std::vector<uint8_t>
    Storage::serialize_var_map(const std::unordered_map<std::string, PrimitiveVariable>& variables);

template bool Storage::deserialize_var_map(
    std::vector<uint8_t>& buffer, uint16_t num_vars,
    std::unordered_map<std::string, Storage::PrimitiveVariable>& variables
);

template std::vector<uint8_t>
    Storage::serialize_var_map(const std::unordered_map<std::string, SerializableVariable>& variables);

template bool Storage::deserialize_var_map(
    std::vector<uint8_t>& buffer, uint16_t num_vars,
    std::unordered_map<std::string, Storage::SerializableVariable>& variables
);
}  // namespace micras::proxy
