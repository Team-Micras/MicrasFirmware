/**
 * @file
 */

#include <algorithm>
#include <cstddef>
#include <cstring>
#include <limits>
#include <stdexcept>

#include "micras/micras.hpp"
#include "micras/sim/micras/pool_variables.hpp"

namespace micras::sim {
namespace {
/**
 * @brief Read a registered variable as a number.
 *
 * @param variable The variable.
 * @return Its value.
 */
double read(const core::Variable& variable) {
    const auto load = [&variable]<typename T>(T) {
        T value{};
        std::memcpy(&value, variable.address, sizeof(value));
        return static_cast<double>(value);
    };

    switch (variable.type) {
        case core::TypeCode::BOOL:
            return load(bool{});
        case core::TypeCode::U8:
            return load(uint8_t{});
        case core::TypeCode::I8:
            return load(int8_t{});
        case core::TypeCode::U16:
            return load(uint16_t{});
        case core::TypeCode::I16:
            return load(int16_t{});
        case core::TypeCode::U32:
            return load(uint32_t{});
        case core::TypeCode::I32:
            return load(int32_t{});
        case core::TypeCode::U64:
            return load(uint64_t{});
        case core::TypeCode::I64:
            return load(int64_t{});
        case core::TypeCode::F32:
            return load(float{});
        case core::TypeCode::F64:
            return load(double{});
        case core::TypeCode::BLOB:
            break;
    }

    return std::numeric_limits<double>::quiet_NaN();
}

/**
 * @brief Get the full name of a variable.
 *
 * @param variable The variable.
 * @return Its prefix and name.
 */
std::string full_name(const core::Variable& variable) {
    return std::string{variable.prefix} + std::string{variable.name};
}

/**
 * @brief Get the robot, or fail when the firmware has not built it.
 *
 * @return The robot.
 */
const Micras& robot() {
    const Micras* micras = Micras::get_instance();

    if (micras == nullptr) {
        throw std::runtime_error("the firmware did not construct its robot during the first tick");
    }

    return *micras;
}
}  // namespace

std::vector<std::string> PoolVariables::names() {
    std::vector<std::string> columns{"state"};
    this->recorded.clear();

    const std::span<const core::Variable> variables = robot().get_variables().all();

    for (std::size_t index = 0; index < variables.size(); index++) {
        const core::Variable& variable = variables[index];

        if (variable.type == core::TypeCode::BLOB) {
            continue;
        }

        std::string column = full_name(variable);
        std::ranges::replace(column, '/', '_');
        columns.push_back(column);
        this->recorded.push_back(static_cast<core::VariableId>(index));
    }

    return columns;
}

void PoolVariables::append(std::vector<CsvCell>& row) {
    const Micras& micras = robot();
    row.emplace_back(static_cast<int64_t>(micras.get_state()));

    for (const core::VariableId id : this->recorded) {
        row.emplace_back(read(micras.get_variables().at(id)));
    }
}

double PoolVariables::value_of(const std::string& name) const {
    const Micras* micras = Micras::get_instance();

    if (micras == nullptr) {
        return std::numeric_limits<double>::quiet_NaN();
    }

    if (name == "state") {
        return micras->get_state();
    }

    const std::optional<core::VariableId> id = micras->get_variables().find(name);
    return id.has_value() ? read(micras->get_variables().at(*id)) : std::numeric_limits<double>::quiet_NaN();
}
}  // namespace micras::sim
