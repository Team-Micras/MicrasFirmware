#ifndef MICRAS_CORE_CONCEPTS_HPP
#define MICRAS_CORE_CONCEPTS_HPP

#include "serializable.hpp"
#include <type_traits>

namespace micras::core {

template <typename T>
concept Fundamental = std::is_fundamental_v<T>;

template <typename T>
concept Serializable = std::is_base_of_v<ISerializable, T>;

}  // namespace micras::core

#endif  // MICRAS_CORE_CONCEPTS_HPP
