/**
 * @file
 */

#ifndef MICRAS_COMM_LINK_HPP
#define MICRAS_COMM_LINK_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

#include "micras/comm/frame.hpp"
#include "micras/comm/group.hpp"
#include "micras/comm/protocol.hpp"
#include "micras/comm/trace.hpp"
#include "micras/core/byte_stream.hpp"
#include "micras/core/variable_pool.hpp"

namespace micras::comm {
/**
 * @brief Result of a command sent over the link.
 */
enum class CommandResult : uint8_t {
    OK = 0,
    UNKNOWN = 1,
    REFUSED = 2
};

/**
 * @brief Interface for whatever turns a command into an action on the robot.
 *
 * @note Commands are edges and writes are levels, which is the distinction that keeps a command
 * from being re-applied on every iteration. The link never holds a command flag of its own.
 */
class ICommandHandler {
public:
    /**
     * @brief Virtual destructor for the ICommandHandler class.
     */
    virtual ~ICommandHandler() = default;

    /**
     * @brief Act on a command.
     *
     * @param code Command to run.
     * @param argument Argument of the command.
     * @return Whether the command ran.
     */
    virtual CommandResult handle_command(uint8_t code, uint32_t argument) = 0;

protected:
    /**
     * @brief Special member functions declared as default.
     */
    ///@{
    ICommandHandler() = default;
    ICommandHandler(const ICommandHandler&) = default;
    ICommandHandler(ICommandHandler&&) = default;
    ICommandHandler& operator=(const ICommandHandler&) = default;
    ICommandHandler& operator=(ICommandHandler&&) = default;
    ///@}
};

/**
 * @brief Session over a byte stream, exposing the variable pool to the outside world.
 */
class Link {
public:
    /**
     * @brief Configuration struct for the link.
     */
    struct Config {
        /**
         * @brief Period of the control loop in microseconds, which is the unit of a group period.
         */
        uint32_t loop_time_us;
    };

    /**
     * @brief Construct a new Link object.
     *
     * @param stream Transport the session runs over.
     * @param pool Variables the session exposes.
     * @param trace Capture buffer the session arms and reads out.
     * @param commands Handler the commands are given to.
     * @param config Configuration for the link.
     */
    Link(
        core::IByteStream& stream, core::VariablePool& pool, Trace& trace, ICommandHandler& commands,
        const Config& config
    );

    /**
     * @brief Take whatever arrived and act on at most one message.
     *
     * @note Limiting this to one message per iteration is what keeps a burst of requests from
     * spending the loop budget, and one message per 125 us is far faster than the link can deliver
     * them anyway.
     *
     * @param robot_is_idle Whether the robot is stopped, which gates the guarded writes.
     */
    void poll(bool robot_is_idle);

    /**
     * @brief Send whatever is due.
     *
     * @note Called every iteration. It costs a few compares when no group is due.
     *
     * @param timestamp_us Current time in microseconds, stamped on the samples.
     */
    void pump(uint32_t timestamp_us);

    /**
     * @brief Send a message to the application.
     *
     * @note Dropped when the transport is full rather than queued, and the number of dropped
     * messages is itself registered in the pool, so a gap is visible instead of silent.
     *
     * @param severity Severity of the message.
     * @param text Message to send.
     */
    void log(Severity severity, std::string_view text);

    /**
     * @brief Register the counters of the link itself.
     *
     * @param pool Pool to register into.
     * @param prefix Prefix of the names.
     */
    void register_variables(core::VariablePool& pool, std::string_view prefix);

private:
    /**
     * @brief Act on one decoded message.
     *
     * @param robot_is_idle Whether the robot is stopped.
     */
    void execute(bool robot_is_idle);

    /**
     * @brief Handlers for each kind of request.
     */
    ///@{
    void on_hello();
    void on_schema_request(Reader& reader);
    void on_group_define(Reader& reader);
    void on_group_enable(Reader& reader);
    void on_credit(Reader& reader);
    void on_write(Reader& reader, bool robot_is_idle);
    void on_read(Reader& reader);
    void on_command(Reader& reader);
    void on_trace_arm(Reader& reader);
    void on_trace_read(Reader& reader);
    ///@}

    /**
     * @brief Send one page of the schema, if one was asked for.
     *
     * @return True if a frame was sent, false otherwise.
     */
    bool send_schema_page();

    /**
     * @brief Send one block of the capture, if one was asked for.
     *
     * @return True if a frame was sent, false otherwise.
     */
    bool send_trace_block();

    /**
     * @brief Send the state of the capture.
     */
    void send_trace_status();

    /**
     * @brief Send an error for a message that could not be acted on.
     *
     * @param code Reason the message was refused.
     * @param context Whatever identifies the offending message.
     */
    void send_error(ErrorCode code, uint16_t context);

    /**
     * @brief Send a frame the application asked for, which is bounded by its own request rate and
     * so is not charged to the credit window.
     *
     * @param type Type of the message.
     * @param payload Payload of the message.
     * @return True if the transport took the frame, false otherwise.
     */
    bool send(MessageType type, std::span<const uint8_t> payload);

    /**
     * @brief Send a frame the robot produced on its own, which the credit window has to allow.
     *
     * @param type Type of the message.
     * @param payload Payload of the message.
     * @return True if the transport took the frame, false otherwise.
     */
    bool send_metered(MessageType type, std::span<const uint8_t> payload);

    // NOLINTBEGIN(*-avoid-const-or-ref-data-members) borrowed for the lifetime of the robot
    core::IByteStream&  stream;
    core::VariablePool& pool;
    Trace&              trace;
    ICommandHandler&    commands;
    // NOLINTEND(*-avoid-const-or-ref-data-members)

    Config config;

    FrameReader reader;

    /**
     * @brief Bytes taken from the transport but not yet fed to the frame reader.
     */
    ///@{
    std::array<uint8_t, 64> staging{};
    std::size_t             staged{};
    std::size_t             consumed{};
    ///@}

    /**
     * @brief Scratch buffers for the message being built and for the frame around it.
     */
    ///@{
    std::array<uint8_t, max_payload_size> payload{};
    std::array<uint8_t, max_frame_size>   frame{};
    ///@}

    std::array<Group, max_groups> groups{};

    /**
     * @brief Bytes the application still allows to be sent on the robot's own initiative.
     */
    int32_t credit{initial_credit};

    /**
     * @brief Index of the next schema entry to send, past the last one when no page is due.
     */
    uint16_t schema_index{};

    /**
     * @brief Offset of the next block of the capture to send, past the end when none is due.
     */
    uint32_t trace_offset{};

    /**
     * @brief Whether a capture is being read out.
     */
    bool trace_dumping{};

    /**
     * @brief Counters worth watching from the application.
     */
    ///@{
    uint32_t dropped_samples{};
    uint32_t dropped_logs{};
    ///@}
};
}  // namespace micras::comm

#endif  // MICRAS_COMM_LINK_HPP
