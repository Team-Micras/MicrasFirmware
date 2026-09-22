/**
 * @file
 */

#ifndef MICRAS_CORE_BYTE_STREAM_HPP
#define MICRAS_CORE_BYTE_STREAM_HPP

#include <cstddef>
#include <cstdint>
#include <span>

namespace micras::core {
/**
 * @brief Interface for a bidirectional stream of bytes.
 *
 * @note The session layer only ever sees this, so the same protocol runs over the radio, over a
 * socket in the simulators and over a pipe in a host test. An interface is used instead of a
 * callable because it allocates nothing and costs one virtual call per batch rather than per byte.
 */
class IByteStream {
public:
    /**
     * @brief Virtual destructor for the IByteStream class.
     */
    virtual ~IByteStream() = default;

    /**
     * @brief Take whatever has already arrived.
     *
     * @param into Buffer to read into.
     * @return Number of bytes read, which is zero when nothing has arrived.
     */
    virtual std::size_t read(std::span<uint8_t> into) = 0;

    /**
     * @brief Queue data to be sent.
     *
     * @param from Data to send.
     * @return Number of bytes accepted, which is either all of them or none.
     */
    virtual std::size_t write(std::span<const uint8_t> from) = 0;

    /**
     * @brief Get how much can be written before the stream refuses data.
     *
     * @return Number of bytes that can be written right now.
     */
    virtual std::size_t writable() const = 0;

protected:
    /**
     * @brief Special member functions declared as default.
     */
    ///@{
    IByteStream() = default;
    IByteStream(const IByteStream&) = default;
    IByteStream(IByteStream&&) = default;
    IByteStream& operator=(const IByteStream&) = default;
    IByteStream& operator=(IByteStream&&) = default;
    ///@}
};
}  // namespace micras::core

#endif  // MICRAS_CORE_BYTE_STREAM_HPP
