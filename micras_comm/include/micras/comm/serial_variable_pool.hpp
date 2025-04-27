#ifndef MICRAS_COMM_SERIAL_VARIABLE_POOL_HPP
#define MICRAS_COMM_SERIAL_VARIABLE_POOL_HPP

#include <cstdint>
#include <string>
#include <type_traits>
#include <vector>
#include <memory>

#include <unordered_map>
#include <map>

#include "serial_variable.hpp"
#include "serializable.hpp"
#include "primitive_serial_variable.hpp"
#include "custom_serial_variable.hpp"

#include <string_view>

namespace micras::comm {
template <typename T>
concept Fundamental = std::is_fundamental_v<T>;

template <typename T>
concept Serializable = std::is_base_of<ISerialVariable, T>::value;

/**
 * @brief
 *
 * @cite https://stackoverflow.com/questions/81870/is-it-possible-to-print-a-variables-type-in-standard-c/64490578#64490578
 *
 * @tparam T
 * @return constexpr auto
 */
template <typename T>
constexpr auto type_name() {
    std::string_view name, prefix, suffix;
#ifdef __clang__
    name = __PRETTY_FUNCTION__;
    prefix = "auto type_name() [T = ";
    suffix = "]";
#elif defined(__GNUC__)
    name = __PRETTY_FUNCTION__;
    prefix = "constexpr auto type_name() [with T = ";
    suffix = "]";
#elif defined(_MSC_VER)
    name = __FUNCSIG__;
    prefix = "auto __cdecl type_name<";
    suffix = ">(void)";
#endif
    name.remove_prefix(prefix.size());
    name.remove_suffix(suffix.size());
    return name;
}


class SerialVariablePool {
public:
    explicit SerialVariablePool();

    template <Fundamental T>
    void add_read_only(const std::string& name, T& data) {
        uint16_t var_id = current_id++;
        this->variables[var_id] = std::make_unique<PrimitiveSerialVariable<T>>(name, &data, true);
    }

    template <Fundamental T>
    void add_write_only(const std::string& name, T& data) {
        uint16_t var_id = current_id++;
        this->variables[var_id] = std::make_unique<PrimitiveSerialVariable<T>>(name, &data, false);
    }

    template <Serializable T>
    void add_read_only(const std::string& name, T& data) {
        uint16_t var_id = current_id++;
        this->variables[var_id] = std::make_unique<CustomSerialVariable<T>>(name, &data, true);
    }

    template <Serializable T>
    void add_write_only(const std::string& name, T& data) {
        uint16_t var_id = current_id++;
        this->variables[var_id] = std::make_unique<CustomSerialVariable<T>>(name, &data, false);
    }

    std::vector<uint8_t> serialize_var_map();

private:
    static uint16_t current_id;

    std::unordered_map<uint16_t, std::unique_ptr<ISerialVariable>> variables;

    static std::vector<uint8_t> serialize_var_map();
};
}  // namespace micras::comm

#endif  // MICRAS_COMM_SERIAL_VARIABLE_POOL_HPP
