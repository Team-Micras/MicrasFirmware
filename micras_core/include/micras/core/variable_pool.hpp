/**
 * @file
 */

#ifndef MICRAS_CORE_VARIABLE_POOL_HPP
#define MICRAS_CORE_VARIABLE_POOL_HPP

#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <string_view>
#include <type_traits>

#include "micras/core/serializable.hpp"

namespace micras::core {
/**
 * @brief Type of a registered variable, as it is written to the flash and to the wire.
 */
enum class TypeCode : uint8_t {
    BOOL = 0,
    U8 = 1,
    I8 = 2,
    U16 = 3,
    I16 = 4,
    U32 = 5,
    I32 = 6,
    U64 = 7,
    I64 = 8,
    F32 = 9,
    F64 = 10,
    BLOB = 11
};

/**
 * @brief Map from a C++ type to the type code that describes it.
 *
 * @note Left undefined for every type without a specialization, so that registering an unsupported
 * type fails to compile instead of being written out as raw bytes nothing can read back.
 *
 * @tparam T Type to map.
 */
template <typename T>
struct TypeCodeOf { };

/**
 * @brief Specializations of the type code map.
 */
///@{
template <>
struct TypeCodeOf<bool> {
    static constexpr TypeCode value{TypeCode::BOOL};
};

template <>
struct TypeCodeOf<uint8_t> {
    static constexpr TypeCode value{TypeCode::U8};
};

template <>
struct TypeCodeOf<int8_t> {
    static constexpr TypeCode value{TypeCode::I8};
};

template <>
struct TypeCodeOf<uint16_t> {
    static constexpr TypeCode value{TypeCode::U16};
};

template <>
struct TypeCodeOf<int16_t> {
    static constexpr TypeCode value{TypeCode::I16};
};

template <>
struct TypeCodeOf<uint32_t> {
    static constexpr TypeCode value{TypeCode::U32};
};

template <>
struct TypeCodeOf<int32_t> {
    static constexpr TypeCode value{TypeCode::I32};
};

template <>
struct TypeCodeOf<uint64_t> {
    static constexpr TypeCode value{TypeCode::U64};
};

template <>
struct TypeCodeOf<int64_t> {
    static constexpr TypeCode value{TypeCode::I64};
};

template <>
struct TypeCodeOf<float> {
    static constexpr TypeCode value{TypeCode::F32};
};

template <>
struct TypeCodeOf<double> {
    static constexpr TypeCode value{TypeCode::F64};
};

template <typename T>
requires std::is_enum_v<T>
struct TypeCodeOf<T> : TypeCodeOf<std::underlying_type_t<T>> { };

///@}

/**
 * @brief Types that can be registered by value, as opposed to through ISerializable.
 *
 * @tparam T Type to check.
 */
template <typename T>
concept Registrable = std::is_trivially_copyable_v<T> and requires { TypeCodeOf<std::remove_cv_t<T>>::value; };

/**
 * @brief What each consumer of the pool is allowed to do with a variable.
 */
struct Access {
    /**
     * @brief Whether the variable can be part of a stream.
     */
    bool stream : 1 {};

    /**
     * @brief Whether the variable can be written from outside the robot.
     */
    bool write : 1 {};

    /**
     * @brief Whether writes are restricted to a stopped robot, ignored when write is not set.
     */
    bool idle : 1 {};

    /**
     * @brief Whether the variable is part of the flash image.
     */
    bool persist : 1 {};
};

/**
 * @brief A variable registered in the pool.
 *
 * @note The name is split in two views so that an owner can be given a prefix at registration
 * without concatenating anything at run time. Both are expected to be string literals, and must
 * outlive the pool.
 */
struct Variable {
    std::string_view prefix;
    std::string_view name;

    /**
     * @brief Where the value lives, or the ISerializable that owns it when the type is BLOB.
     */
    void* address{};

    /**
     * @brief Number of bytes of the value, zero for a BLOB, whose size is only known once it is
     * serialized.
     */
    uint16_t size{};

    TypeCode type{};
    Access   access{};
};

/**
 * @brief Index of a variable in the pool, and the identifier used on the wire.
 */
using VariableId = uint16_t;

/**
 * @brief Reflection over the variables the rest of the firmware is willing to expose.
 *
 * @note This class knows nothing about flash, packets or schedules. It is a flat array of plain
 * descriptors, and every consumer walks it and decides for itself what the bytes are for.
 *
 * @note The pool does not own the storage of the values, only their addresses, so a registered
 * object must outlive the pool.
 */
class VariablePool {
public:
    /**
     * @brief Result of an attempt to write a variable from outside the robot.
     */
    enum class WriteStatus : uint8_t {
        OK = 0,
        NO_SUCH_ID = 1,
        READ_ONLY = 2,
        NEEDS_IDLE = 3,
        WRONG_SIZE = 4
    };

    /**
     * @brief Identifier returned when a variable could not be registered.
     */
    static constexpr VariableId invalid_id{0xFFFF};

    /**
     * @brief Construct a new VariablePool object over storage owned by the caller.
     *
     * @param storage Array the descriptors are written into, which bounds the number of variables.
     */
    explicit VariablePool(std::span<Variable> storage);

    /**
     * @brief Register a writable variable.
     *
     * @tparam T Type of the variable.
     * @param prefix Prefix of the name, usually identifying the owner.
     * @param name Name of the variable inside its owner.
     * @param value Reference to the variable, which must outlive the pool.
     * @param access What the consumers of the pool may do with the variable.
     * @return Identifier of the variable, or invalid_id if the pool is full.
     */
    template <Registrable T>
    VariableId add(std::string_view prefix, std::string_view name, T& value, Access access);

    /**
     * @brief Register a read only variable.
     *
     * @note The write flag is cleared whatever the caller asked for, so that constness is enough to
     * state the intent at the registration site.
     *
     * @tparam T Type of the variable.
     * @param prefix Prefix of the name, usually identifying the owner.
     * @param name Name of the variable inside its owner.
     * @param value Reference to the variable, which must outlive the pool.
     * @param access What the consumers of the pool may do with the variable.
     * @return Identifier of the variable, or invalid_id if the pool is full.
     */
    template <Registrable T>
    VariableId add(std::string_view prefix, std::string_view name, const T& value, Access access);

    /**
     * @brief Register an object with its own encoding.
     *
     * @note Serializing costs a heap allocation, so a blob is meant for composite state that is
     * saved and loaded, never for something sampled at loop rate.
     *
     * @param prefix Prefix of the name, usually identifying the owner.
     * @param name Name of the variable inside its owner.
     * @param object Reference to the object, which must outlive the pool.
     * @param access What the consumers of the pool may do with the variable.
     * @return Identifier of the variable, or invalid_id if the pool is full.
     */
    VariableId add(std::string_view prefix, std::string_view name, ISerializable& object, Access access);

    /**
     * @brief Find a variable by its full name.
     *
     * @param full_name Prefix and name of the variable, concatenated.
     * @return Identifier of the variable, or no value if it is not registered.
     */
    std::optional<VariableId> find(std::string_view full_name) const;

    /**
     * @brief Get a registered variable.
     *
     * @param id Identifier of the variable, which must be valid.
     * @return The descriptor of the variable.
     */
    const Variable& at(VariableId id) const;

    /**
     * @brief Get every registered variable, in registration order.
     *
     * @return View over the descriptors.
     */
    std::span<const Variable> all() const;

    /**
     * @brief Copy the current value of a variable.
     *
     * @note Only defined for variables that are not blobs, which are read through their own
     * interface instead.
     *
     * @param id Identifier of the variable, which must be valid.
     * @param into Buffer to copy into, which must hold at least the size of the variable.
     * @return Number of bytes copied, zero if the variable is a blob or the buffer is too small.
     */
    uint16_t read(VariableId id, std::span<uint8_t> into) const;

    /**
     * @brief Write a variable from outside the robot.
     *
     * @param id Identifier of the variable.
     * @param bytes New value of the variable.
     * @param robot_is_idle Whether the robot is currently stopped.
     * @return Whether the write was performed, and why not otherwise.
     */
    WriteStatus write(VariableId id, std::span<const uint8_t> bytes, bool robot_is_idle);

    /**
     * @brief Compute a hash over the layout of the pool.
     *
     * @note Identifiers are registration order, so adding one variable shifts every later one. An
     * application holding a cached schema would then plot the wrong signal with nothing to warn it.
     * Comparing this hash against the one the schema was fetched with is that warning.
     *
     * @note Only has to change whenever the schema does, so it is an FNV-1a hash rather than
     * anything with error detection properties. Nothing is being corrected here, and the schema it
     * stands for arrives over a link that checks its own frames.
     *
     * @return Hash over the name, type and access flags of every variable, in order.
     */
    uint32_t schema_hash() const;

private:
    /**
     * @brief Reserve the next descriptor.
     *
     * @return Pointer to the descriptor, or nullptr if the pool is full.
     */
    Variable* next();

    /**
     * @brief Descriptors of the registered variables, owned by the caller.
     */
    std::span<Variable> storage;

    /**
     * @brief Number of variables registered so far.
     */
    VariableId count{};
};

namespace detail {
/**
 * @brief Storage for a variable pool, held in a base class so that it is already constructed when
 * the pool base is given a view over it.
 *
 * @tparam N Maximum number of variables.
 */
template <std::size_t N>
struct VariableStorage {
    std::array<Variable, N> variables{};
};
}  // namespace detail

/**
 * @brief A variable pool owning storage for a fixed number of variables.
 *
 * @tparam N Maximum number of variables.
 */
template <std::size_t N>
// NOLINTNEXTLINE(misc-multiple-inheritance) the first base only exists to hold the storage
class TVariablePool : private detail::VariableStorage<N>, public VariablePool {
public:
    /**
     * @brief Construct a new TVariablePool object.
     */
    TVariablePool() : VariablePool{this->variables} { }
};
}  // namespace micras::core

#include "micras/core/impl/variable_pool.tpp"  // IWYU pragma: export

#endif  // MICRAS_CORE_VARIABLE_POOL_HPP
