/**
 * @file
 */

#ifndef MICRAS_COMM_LINK_HPP
#define MICRAS_COMM_LINK_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string_view>

#include "micras/comm/credit_window.hpp"
#include "micras/comm/frame.hpp"
#include "micras/comm/protocol.hpp"
#include "micras/core/byte_stream.hpp"
#include "micras/core/variable_pool.hpp"

namespace micras::comm {
/**
 * @brief Result of a command sent over the link.
 */
enum class CommandResult : uint8_t {
    OK = 0,
    UNKNOWN = 1,
    REFUSED = 2,
    DEFERRED = 3
};

/**
 * @brief Answer to a command, sent back as soon as the command arrives.
 *
 * @note The reasons belong to the robot, like the command codes do, so the link carries them as
 * plain bytes and zero is the only value it gives a meaning to: no reason.
 */
// NOLINTNEXTLINE(cppcoreguidelines-pro-type-member-init) no default result, so that a missing one is a warning
struct CommandReply {
    /**
     * @brief Whether the command ran, was refused, or was accepted to run later.
     */
    CommandResult result;

    /**
     * @brief Why the command was refused or deferred, or zero.
     */
    uint8_t reason{};
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
     * @return Whether the command ran, and why not otherwise.
     */
    virtual CommandReply handle_command(uint8_t code, uint32_t argument) = 0;

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

        /**
         * @brief Name the robot introduces itself with, which the application chooses its view of
         * the robot by. Expected to be a string literal.
         */
        std::string_view robot_name;

        /**
         * @brief Number that tells this board and this boot apart, mixed into the boot identifier.
         */
        uint32_t boot_seed;
    };

    /**
     * @brief Construct a new Link object.
     *
     * @param stream Transport the session runs over.
     * @param pool Variables the session exposes.
     * @param commands Handler the commands are given to.
     * @param config Configuration for the link.
     */
    Link(core::IByteStream& stream, core::VariablePool& pool, ICommandHandler& commands, const Config& config);

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
     * @note Charged to the credit window like a sample, since the robot sends it on its own
     * initiative. A message the window has no room for is held and sent first thing when there is,
     * ahead of the samples, which would otherwise take every byte of credit as it comes back. Only
     * one is held: a message that finds another one waiting is dropped, and the number of dropped
     * messages is itself registered in the pool, so a gap is visible instead of silent.
     *
     * @param severity Severity of the message.
     * @param timestamp_us Time the message is about, on the clock the samples are stamped with.
     * @param text Message to send, cut to what fits in one frame.
     */
    void log(Severity severity, uint32_t timestamp_us, std::string_view text);

    /**
     * @brief Register the counters of the link itself.
     *
     * @param pool Pool to register into.
     * @param prefix Prefix of the names.
     */
    void register_variables(core::VariablePool& pool, std::string_view prefix);

private:
    /**
     * @brief A set of variables sampled in the same loop iteration and sent under one timestamp.
     *
     * @note Sampling several variables under one header and one timestamp is not only cheaper on a
     * link this slow, it is the only way the samples mean anything together. A response plotted
     * against a setpoint captured two iterations later is a plot of the loop plus an unknown delay.
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
    };

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
    void on_ping();
    void on_schema_request(Reader& reader);
    void on_group_define(Reader& reader);
    void on_group_enable(Reader& reader);
    void on_credit(Reader& reader);
    void on_write(Reader& reader, bool robot_is_idle);
    void on_read(Reader& reader);
    void on_command(Reader& reader);
    ///@}

    /**
     * @brief Send one page of the schema, if one was asked for.
     */
    void send_schema_page();

    /**
     * @brief Send the log message that is being held, if the window has room for it now.
     */
    void send_held_log();

    /**
     * @brief Copy the room left in the window into the counter the pool exposes.
     */
    void refresh_credit();

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
     * @note The frame is sent only if the bytes in flight, counting it, still fit in the window.
     *
     * @param type Type of the message.
     * @param payload Payload of the message.
     * @return True if the transport took the frame, false otherwise.
     */
    bool send_metered(MessageType type, std::span<const uint8_t> payload);

    // NOLINTBEGIN(*-avoid-const-or-ref-data-members) borrowed for the lifetime of the robot
    core::IByteStream&  stream;
    core::VariablePool& pool;
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
     * @brief Bytes sent on the robot's own initiative against what the application consumed.
     */
    CreditWindow window;

    /**
     * @brief Bytes the window still allows, as the pool exposes it.
     */
    int32_t credit{credit_window};

    /**
     * @brief Payload of the log message waiting for room in the window, and its size, zero when
     * none is.
     */
    ///@{
    std::array<uint8_t, max_payload_size> held_log{};
    std::size_t                           held_log_size{};
    ///@}

    /**
     * @brief Time of the last pump, in microseconds.
     */
    uint32_t last_timestamp_us{};

    /**
     * @brief Identifier of this boot, mixed from the boot seed and the clock when the first HELLO
     * arrives.
     *
     * @note The board enables no source of randomness, and everything the robot does at startup
     * runs from the same clock, so the seed alone could come out the same on two boots. The moment
     * an application first connects differs, to the microsecond. Every later HELLO of the same boot
     * gets the same identifier, which is how the application tells a reconnection from a reboot.
     */
    std::optional<uint32_t> boot_id;

    /**
     * @brief Value of the schema index when no page is due.
     *
     * @note Past any pool, rather than past the pool as it is when the link is constructed, since
     * the owner of the link usually registers the variables after constructing it, and a page
     * nobody asked for would then be sent at boot.
     */
    static constexpr uint16_t no_schema_page{UINT16_MAX};

    /**
     * @brief Index of the next schema entry to send, past the last one when no page is due.
     */
    uint16_t schema_index{no_schema_page};

    /**
     * @brief Counters worth watching from the application.
     */
    ///@{
    uint32_t dropped_samples{};
    uint32_t dropped_logs{};
    uint32_t discarded_frames{};
    ///@}
};
}  // namespace micras::comm

#endif  // MICRAS_COMM_LINK_HPP
