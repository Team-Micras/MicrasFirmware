#ifndef MICRAS_COMM_SERIAL_VARIABLE_POOL_HPP
#define MICRAS_COMM_SERIAL_VARIABLE_POOL_HPP

#include <cstdint>
#include <string>
#include <type_traits>
#include <vector>
#include <unordered_map>
#include <map>

#include "serializable.hpp"

#include <string_view>

namespace micras::comm {
template <typename T>
concept Fundamental = std::is_fundamental_v<T>;

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

    void add_read_only(const std::string& name, const proxy::ISerializable& data);

    template <Fundamental T>
    void add_read_only(const std::string& name, const T& data) {
        this->add_variable<T>(name, true, data);
    }

    void add_write_only(const std::string& name, const proxy::ISerializable& data);

    template <Fundamental T>
    void add_write_only(const std::string& name, const T& data) {
        this->add_variable<T>(name, false, data);
    }

    std::vector<uint8_t> serialize_var_map();

private:
    static uint16_t current_id;

    struct PrimitiveVariable {
        const void* ram_pointer{nullptr};
        std::string name{};
        std::string type{};
        uint16_t    size{};
        bool        is_read_only{};
    };

    struct SerializableVariable {
        const proxy::ISerializable* ram_pointer{nullptr};
        std::string                 name{};
        std::string                 type{};
        uint16_t                    size{};
        bool                        is_read_only{};
    };

    /**
     * @brief Map of primitive variables.
     */
    std::unordered_map<uint16_t, PrimitiveVariable> primitives;

    /**
     * @brief Map of serializable variables.
     */
    std::unordered_map<uint16_t, SerializableVariable> serializables;

    template <Fundamental T>
    void add_variable(const std::string& name, bool is_read_only, const T& data) {
        uint16_t var_id = current_id++;
        this->primitives[var_id].ram_pointer = &data;
        this->primitives[var_id].name       = name;
        this->primitives[var_id].type       = type_name<T>();
        this->primitives[var_id].size       = sizeof(T);
        this->primitives[var_id].is_read_only = is_read_only;
    }

    void add_variable(const std::string& name, bool is_read_only, const proxy::ISerializable& data);

    template <typename T>
    static std::vector<uint8_t> serialize_var_map(const std::unordered_map<uint16_t, T>& variables);
};
}  // namespace micras::comm

#endif  // MICRAS_COMM_SERIAL_VARIABLE_POOL_HPP
