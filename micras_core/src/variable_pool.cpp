/**
 * @file
 */

#include <bit>
#include <cstdint>
#include <cstring>
#include <optional>
#include <span>
#include <string_view>
#include <utility>

#include "micras/core/serializable.hpp"
#include "micras/core/variable_pool.hpp"

namespace micras::core {
static uint32_t mix(uint32_t hash, uint8_t byte) {
    return (hash ^ byte) * 16777619U;
}

static uint32_t mix(uint32_t hash, std::string_view text) {
    for (const char character : text) {
        hash = mix(hash, static_cast<uint8_t>(character));
    }

    return hash;
}

VariablePool::VariablePool(std::span<Variable> storage) : storage{storage} { }

Variable* VariablePool::next() {
    if (this->count >= this->storage.size()) {
        return nullptr;
    }

    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-avoid-unchecked-container-access)
    return &this->storage[this->count];
}

VariableId VariablePool::add(std::string_view prefix, std::string_view name, ISerializable& object, Access access) {
    Variable* variable = this->next();

    if (variable == nullptr) {
        return invalid_id;
    }

    access.stream = false;

    *variable = {
        .prefix = prefix,
        .name = name,
        .address = static_cast<void*>(&object),
        .size = 0,
        .type = TypeCode::BLOB,
        .access = access,
    };

    return this->count++;
}

std::optional<VariableId> VariablePool::find(std::string_view full_name) const {
    for (VariableId id = 0; id < this->count; id++) {
        const Variable& variable = this->at(id);

        if (full_name.size() == variable.prefix.size() + variable.name.size() and
            full_name.starts_with(variable.prefix) and full_name.ends_with(variable.name)) {
            return id;
        }
    }

    return std::nullopt;
}

const Variable& VariablePool::at(VariableId id) const {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-avoid-unchecked-container-access)
    return this->storage[id];
}

std::span<const Variable> VariablePool::all() const {
    return this->storage.first(this->count);
}

uint16_t VariablePool::read(VariableId id, std::span<uint8_t> into) const {
    if (id >= this->count) {
        return 0;
    }

    const Variable& variable = this->at(id);

    if (variable.type == TypeCode::BLOB or into.size() < variable.size) {
        return 0;
    }

    std::memcpy(into.data(), variable.address, variable.size);
    return variable.size;
}

// NOLINTNEXTLINE(readability-make-member-function-const) it writes through the stored addresses
VariablePool::WriteStatus VariablePool::write(VariableId id, std::span<const uint8_t> bytes, bool robot_is_idle) {
    if (id >= this->count) {
        return WriteStatus::NO_SUCH_ID;
    }

    const Variable& variable = this->at(id);

    if (not variable.access.write) {
        return WriteStatus::READ_ONLY;
    }

    if (variable.access.idle and not robot_is_idle) {
        return WriteStatus::NEEDS_IDLE;
    }

    if (variable.type == TypeCode::BLOB) {
        static_cast<ISerializable*>(variable.address)->deserialize(bytes.data(), bytes.size());
        return WriteStatus::OK;
    }

    if (bytes.size() != variable.size) {
        return WriteStatus::WRONG_SIZE;
    }

    std::memcpy(variable.address, bytes.data(), variable.size);
    return WriteStatus::OK;
}

uint32_t VariablePool::schema_hash() const {
    uint32_t hash = 2166136261U;

    for (const Variable& variable : this->all()) {
        hash = mix(hash, variable.prefix);
        hash = mix(hash, variable.name);
        hash = mix(hash, std::to_underlying(variable.type));
        hash = mix(hash, std::bit_cast<uint8_t>(variable.access));
    }

    return hash;
}
}  // namespace micras::core
