/**
 * @file
 */

#ifndef MICRAS_CORE_VARIABLE_POOL_TPP
#define MICRAS_CORE_VARIABLE_POOL_TPP

#include <bit>
#include <string_view>
#include <type_traits>

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

    // The pool holds one mutable address for every kind of variable, so that no consumer has to
    // branch on constness. Casting it away is safe because this overload has just cleared the only
    // flag through which anything can write.
    *variable = {
        .prefix = prefix,
        .name = name,
        .address = const_cast<T*>(&value),  // NOLINT(cppcoreguidelines-pro-type-const-cast)
        .size = sizeof(T),
        .type = TypeCodeOf<std::remove_cv_t<T>>::value,
        .access = access,
    };

    return this->count++;
}
}  // namespace micras::core

#endif  // MICRAS_CORE_VARIABLE_POOL_TPP
