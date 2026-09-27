/**
 * @file
 *
 * @brief The firmware's registered variables, read straight from its pool.
 */

#ifndef MICRAS_SIM_MICRAS_POOL_VARIABLES_HPP
#define MICRAS_SIM_MICRAS_POOL_VARIABLES_HPP

#include <string>
#include <vector>

#include "micras/core/variable_pool.hpp"
#include "micras/sim/core/variable_source.hpp"
#include "micras/sim/recording/column_source.hpp"

namespace micras::sim {
/**
 * @brief Records and looks up every variable the firmware registers, and its state.
 *
 * @note Read through Micras::get_variables() while the firmware is parked, so
 *       every value of a row belongs to the same instant. Blobs, such as the
 *       maze, are skipped. The state machine's state is
 *       added as "state", which the pool does not hold. Column names are the
 *       variables' full names with slashes as underscores: "pose/x" is pose_x.
 */
class PoolVariables : public ColumnSource, public VariableSource {
public:
    /**
     * @brief Get the column names.
     *
     * @note Throws when the firmware has not constructed its robot yet.
     *
     * @return "state" and one column per registered variable.
     */
    std::vector<std::string> names() override;

    /**
     * @brief Append every variable's current value.
     *
     * @param row Row being built.
     */
    void append(std::vector<CsvCell>& row) override;

    /**
     * @brief Get a variable's current value.
     *
     * @param name Full name, such as "pose/x", or "state".
     * @return The value, or NaN before the robot exists or for unknown names.
     */
    double value_of(const std::string& name) const override;

private:
    /**
     * @brief Ids of the recorded variables, in column order.
     */
    std::vector<core::VariableId> recorded;
};
}  // namespace micras::sim

#endif  // MICRAS_SIM_MICRAS_POOL_VARIABLES_HPP
