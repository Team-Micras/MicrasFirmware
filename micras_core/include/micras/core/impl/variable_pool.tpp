/**
 * @file
 */

#ifndef MICRAS_CORE_VARIABLE_POOL_TPP
#define MICRAS_CORE_VARIABLE_POOL_TPP

#include <bit>
#include <string_view>
#include <type_traits>

#include "micras/core/serializable.hpp"

namespace micras::core {
template <Registrable T>
VariableId VariablePool::add(std::string_view prefix, std::string_view name, T& value, Access access) {
    Variable* variable = this->next();

    if (variable == nullptr) {
        return invalid_id;
    }

    *variable = {
        .prefix = prefix,
        .name = name,
        .address = static_cast<void*>(&value),
        .size = sizeof(T),
        .type = TypeCodeOf<std::remove_cv_t<T>>::value,
        .access = access,
        .type_tag = {},
    };

    return this->count++;
}

template <Registrable T>
VariableId VariablePool::add(std::string_view prefix, std::string_view name, const T& value, Access access) {
    access.write = false;
    access.idle = false;

    Variable* variable = this->next();

    if (variable == nullptr) {
        return invalid_id;
    }

    *variable = {
        .prefix = prefix,
        .name = name,
        .address = const_cast<T*>(&value),  // NOLINT(cppcoreguidelines-pro-type-const-cast)
        .size = sizeof(T),
        .type = TypeCodeOf<std::remove_cv_t<T>>::value,
        .access = access,
        .type_tag = {},
    };

    return this->count++;
}

template <Serializable T>
VariableId VariablePool::add(std::string_view prefix, std::string_view name, T& object, Access access) {
    access.stream = false;

    Variable* variable = this->next();

    if (variable == nullptr) {
        return invalid_id;
    }

    *variable = {
        .prefix = prefix,
        .name = name,
        .address = static_cast<void*>(static_cast<ISerializable*>(&object)),
        .size = 0,
        .type = TypeCode::BLOB,
        .access = access,
        .type_tag = T::type_tag,
    };

    return this->count++;
}
}  // namespace micras::core

#endif  // MICRAS_CORE_VARIABLE_POOL_TPP
