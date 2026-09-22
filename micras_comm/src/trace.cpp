/**
 * @file
 */

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>

#include "micras/comm/group.hpp"
#include "micras/comm/protocol.hpp"
#include "micras/comm/trace.hpp"
#include "micras/core/variable_pool.hpp"

namespace micras::comm {
Trace::Trace(const core::VariablePool& pool, std::span<uint8_t> ring) : pool{pool}, ring{ring} { }

bool Trace::arm(const Group& group, uint8_t pre_trigger, TriggerType trigger, core::VariableId id, float threshold) {
    if (group.count == 0 or group.sample_size == 0 or group.sample_size > this->ring.size()) {
        return false;
    }

    if (trigger == TriggerType::ABOVE or trigger == TriggerType::BELOW) {
        if (id >= this->pool.all().size() or this->pool.at(id).type != core::TypeCode::F32) {
            return false;
        }
    }

    this->group = group;
    this->capacity = this->ring.size() / group.sample_size;
    this->requested_pre = std::min(this->capacity * std::min<uint8_t>(pre_trigger, 100) / 100, this->capacity - 1);
    this->trigger = trigger;
    this->watched = id;
    this->threshold = threshold;

    this->filled = 0;
    this->head = 0;
    this->kept_pre = 0;
    this->remaining = 0;
    this->counter = 0;
    this->has_last_value = false;
    this->current_state = TraceState::ARMED;

    return true;
}

void Trace::fire() {
    if (this->current_state == TraceState::ARMED and this->trigger == TriggerType::COMMAND) {
        this->kept_pre = std::min(this->filled, this->requested_pre);
        this->remaining = this->capacity - this->kept_pre;
        this->current_state = TraceState::TRIGGERED;
    }
}

void Trace::stop() {
    this->current_state = TraceState::IDLE;
}

bool Trace::triggered_by_threshold() {
    float value = 0.0F;
    std::memcpy(&value, this->pool.at(this->watched).address, sizeof(float));

    const bool crossed = this->has_last_value and (this->trigger == TriggerType::ABOVE ?
                                                       this->last_value <= this->threshold and value > this->threshold :
                                                       this->last_value >= this->threshold and value < this->threshold);

    this->last_value = value;
    this->has_last_value = true;

    return crossed;
}

void Trace::store() {
    const std::size_t offset = static_cast<std::size_t>(this->head) * this->group.sample_size;
    this->group.sample(this->pool, this->ring.subspan(offset, this->group.sample_size));

    this->head = (this->head + 1) % this->capacity;
    this->filled = std::min(this->filled + 1, this->capacity);
}

void Trace::sample(uint32_t timestamp_us) {
    if (this->current_state != TraceState::ARMED and this->current_state != TraceState::TRIGGERED) {
        return;
    }

    if (this->current_state == TraceState::ARMED) {
        if (this->trigger == TriggerType::IMMEDIATE or
            ((this->trigger == TriggerType::ABOVE or this->trigger == TriggerType::BELOW) and
             this->triggered_by_threshold())) {
            this->kept_pre = std::min(this->filled, this->requested_pre);
            this->remaining = this->capacity - this->kept_pre;
            this->current_state = TraceState::TRIGGERED;
        }
    }

    if (this->counter > 0) {
        this->counter--;
        return;
    }

    this->counter = this->group.period - 1;

    if (this->current_state == TraceState::TRIGGERED and this->remaining == this->capacity - this->kept_pre) {
        this->trigger_timestamp = timestamp_us;
    }

    this->store();

    if (this->current_state == TraceState::TRIGGERED and --this->remaining == 0) {
        this->current_state = TraceState::FULL;
    }
}

TraceState Trace::state() const {
    return this->current_state;
}

uint32_t Trace::held() const {
    return this->current_state == TraceState::FULL ? this->filled : 0;
}

uint32_t Trace::pre_trigger() const {
    return this->kept_pre;
}

uint16_t Trace::sample_size() const {
    return this->group.sample_size;
}

uint16_t Trace::period() const {
    return this->group.period;
}

uint32_t Trace::timestamp() const {
    return this->trigger_timestamp;
}

std::size_t Trace::read(uint32_t offset, std::span<uint8_t> into) const {
    const std::size_t usable = static_cast<std::size_t>(this->capacity) * this->group.sample_size;
    const std::size_t total = static_cast<std::size_t>(this->held()) * this->group.sample_size;
    const std::size_t from = offset;

    if (from >= total) {
        return 0;
    }

    const std::size_t oldest = static_cast<std::size_t>(this->head) * this->group.sample_size;
    const std::size_t size = std::min(into.size(), total - from);
    const std::size_t start = (oldest + from) % usable;
    const std::size_t first = std::min(size, usable - start);

    // NOLINTBEGIN(cppcoreguidelines-pro-bounds-avoid-unchecked-container-access) bounded just above
    std::memcpy(into.data(), &this->ring[start], first);

    if (first < size) {
        std::memcpy(&into[first], this->ring.data(), size - first);
    }
    // NOLINTEND(cppcoreguidelines-pro-bounds-avoid-unchecked-container-access)

    return size;
}
}  // namespace micras::comm
