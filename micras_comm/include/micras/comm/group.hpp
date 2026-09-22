/**
 * @file
 */

#ifndef MICRAS_COMM_GROUP_HPP
#define MICRAS_COMM_GROUP_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "micras/comm/protocol.hpp"
#include "micras/core/variable_pool.hpp"

namespace micras::comm {
/**
 * @brief A set of variables captured in the same loop iteration.
 *
 * @note Sampling several variables under one header and one timestamp is not only cheaper on a link
 * this slow, it is the only way the samples mean anything together. A response plotted against a
 * setpoint captured two iterations later is a plot of the loop plus an unknown delay.
 */
struct Group {
    /**
     * @brief Variables of the group, in the order their values are packed.
     */
    std::array<core::VariableId, max_group_variables> ids{};

    /**
     * @brief Number of variables in the group.
     */
    uint8_t count{};

    /**
     * @brief Number of loop iterations between two samples.
     */
    uint16_t period{1};

    /**
     * @brief Number of bytes of the values of one sample.
     */
    uint16_t sample_size{};

    /**
     * @brief Iterations left until the next sample.
     */
    uint16_t counter{};

    /**
     * @brief Number of samples taken, which lets the application see the ones that were dropped.
     */
    uint16_t sequence{};

    /**
     * @brief Whether the group is being sent.
     */
    bool enabled{};

    /**
     * @brief Copy the current value of every variable of the group.
     *
     * @param pool Pool the variables belong to.
     * @param into Buffer to copy into.
     * @return Number of bytes copied, zero if the buffer is too small.
     */
    std::size_t sample(const core::VariablePool& pool, std::span<uint8_t> into) const;
};
}  // namespace micras::comm

#endif  // MICRAS_COMM_GROUP_HPP
