/**
 * @file
 */

#include <cstddef>
#include <cstdint>
#include <span>

#include "micras/comm/group.hpp"
#include "micras/core/variable_pool.hpp"

namespace micras::comm {
std::size_t Group::sample(const core::VariablePool& pool, std::span<uint8_t> into) const {
    if (into.size() < this->sample_size) {
        return 0;
    }

    std::size_t offset = 0;

    for (uint8_t index = 0; index < this->count; index++) {
        offset += pool.read(this->ids.at(index), into.subspan(offset));
    }

    return offset;
}
}  // namespace micras::comm
