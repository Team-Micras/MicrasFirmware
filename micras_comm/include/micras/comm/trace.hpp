/**
 * @file
 */

#ifndef MICRAS_COMM_TRACE_HPP
#define MICRAS_COMM_TRACE_HPP

#include <cstddef>
#include <cstdint>
#include <span>

#include "micras/comm/group.hpp"
#include "micras/comm/protocol.hpp"
#include "micras/core/variable_pool.hpp"

namespace micras::comm {
/**
 * @brief Full rate capture of a group into a ring in RAM.
 *
 * @note The link is between twenty and a hundred times too slow to carry the control loop, so the
 * only way to see what happened inside a fast run is to record it at loop rate and read it out
 * afterwards. Capturing is a copy into a ring and nothing else, and nothing goes on the wire until
 * the capture has stopped.
 *
 * @note Samples carry no timestamp of their own. The period is fixed at arming, so the instant of
 * sample i is the trigger timestamp plus i times the period, which is four bytes per sample saved
 * on the only path where they would be paid at loop rate.
 */
class Trace {
public:
    /**
     * @brief Construct a new Trace object.
     *
     * @param pool Pool the captured variables belong to.
     * @param ring Memory to capture into, which bounds how long a capture can be.
     */
    Trace(const core::VariablePool& pool, std::span<uint8_t> ring);

    /**
     * @brief Arm a capture.
     *
     * @param group Group to capture, which must already be defined.
     * @param pre_trigger Share of the capture kept from before the trigger, in percent.
     * @param trigger What starts the capture.
     * @param id Variable watched by a threshold trigger.
     * @param threshold Value the watched variable has to cross.
     * @return True if the capture was armed, false if the group does not fit or the trigger makes
     * no sense for the variable it watches.
     */
    bool arm(const Group& group, uint8_t pre_trigger, TriggerType trigger, core::VariableId id, float threshold);

    /**
     * @brief Start a capture waiting on a command trigger.
     */
    void fire();

    /**
     * @brief Abandon the current capture.
     */
    void stop();

    /**
     * @brief Take one sample, if a capture is running.
     *
     * @note Called every loop iteration. It costs a compare when nothing is armed.
     *
     * @param timestamp_us Current time in microseconds, recorded at the trigger.
     */
    void sample(uint32_t timestamp_us);

    /**
     * @brief Get the state of the capture.
     *
     * @return Current state.
     */
    TraceState state() const;

    /**
     * @brief Get how many samples are held.
     *
     * @return Number of samples that can be read.
     */
    uint32_t held() const;

    /**
     * @brief Get how many of the held samples precede the trigger.
     *
     * @return Number of samples taken before the trigger fired.
     */
    uint32_t pre_trigger() const;

    /**
     * @brief Get the number of bytes of one sample.
     *
     * @return Size of a sample in bytes.
     */
    uint16_t sample_size() const;

    /**
     * @brief Get the period of the capture.
     *
     * @return Number of loop iterations between two samples.
     */
    uint16_t period() const;

    /**
     * @brief Get the time the trigger fired.
     *
     * @return Timestamp of the first sample after the trigger, in microseconds.
     */
    uint32_t timestamp() const;

    /**
     * @brief Copy captured bytes, oldest first.
     *
     * @param offset Byte offset into the capture, counting from the oldest sample held.
     * @param into Buffer to copy into.
     * @return Number of bytes copied.
     */
    std::size_t read(uint32_t offset, std::span<uint8_t> into) const;

private:
    /**
     * @brief Check whether a threshold trigger has just been crossed.
     *
     * @return True if the capture should start now, false otherwise.
     */
    bool triggered_by_threshold();

    /**
     * @brief Write the current values of the group at the head of the ring.
     */
    void store();

    // NOLINTNEXTLINE(*-avoid-const-or-ref-data-members) borrowed for the lifetime of the robot
    const core::VariablePool& pool;

    std::span<uint8_t> ring;

    Group group;

    /**
     * @brief Number of samples the ring can hold.
     */
    uint32_t capacity{};

    /**
     * @brief Number of samples written since the capture was armed, saturated at the capacity.
     */
    uint32_t filled{};

    /**
     * @brief Index of the sample that will be written next.
     */
    uint32_t head{};

    /**
     * @brief Number of samples still to take before the capture is complete.
     */
    uint32_t remaining{};

    /**
     * @brief Number of samples requested from before the trigger, and how many were actually kept.
     */
    ///@{
    uint32_t requested_pre{};
    uint32_t kept_pre{};
    ///@}

    /**
     * @brief Iterations left until the next sample.
     */
    uint16_t counter{};

    core::VariableId watched{};
    float            threshold{};
    float            last_value{};
    bool             has_last_value{};

    TriggerType trigger{TriggerType::IMMEDIATE};
    TraceState  current_state{TraceState::IDLE};

    /**
     * @brief Time of the first sample taken after the trigger.
     */
    uint32_t trigger_timestamp{};
};
}  // namespace micras::comm

#endif  // MICRAS_COMM_TRACE_HPP
