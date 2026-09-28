/**
 * @file
 */

#ifndef MICRAS_COMM_CREDIT_WINDOW_HPP
#define MICRAS_COMM_CREDIT_WINDOW_HPP

#include <cstddef>
#include <cstdint>

namespace micras::comm {
/**
 * @brief Accounting of the bytes the robot sends on its own initiative against what the
 * application says it consumed.
 *
 * @note Both sides count cumulative totals since HELLO instead of exchanging increments, so that a
 * CREDIT lost on the way costs nothing: the next one carries everything the lost one did. The
 * totals wrap around at 32 bits, and the difference of two unsigned totals is right across the wrap
 * for as long as it stays below half the range, which the window guarantees by far.
 */
class CreditWindow {
public:
    /**
     * @brief Forget both totals, as a new session starts.
     */
    void reset();

    /**
     * @brief Check if a frame can be sent without overrunning the window.
     *
     * @param size Number of bytes of the frame.
     * @return True if the bytes in flight, counting the frame, fit in the window.
     */
    bool allows(std::size_t size) const;

    /**
     * @brief Count a frame that was sent.
     *
     * @param size Number of bytes of the frame.
     */
    void charge(std::size_t size);

    /**
     * @brief Take the total the application says it consumed.
     *
     * @note A total behind the last one taken arrived late and is ignored. A total ahead of what
     * was sent is taken as everything sent, so that an application that cannot tell a frame that
     * counts from one that does not, such as one it could not decode, may count it anyway.
     *
     * @param consumed_total Metered bytes the application consumed since HELLO, wrapping around.
     */
    void acknowledge(uint32_t consumed_total);

    /**
     * @brief Get the number of bytes sent that the application has not consumed yet.
     *
     * @return The bytes in flight.
     */
    uint32_t outstanding() const;

    /**
     * @brief Get the number of bytes the window still allows.
     *
     * @return The room left in the window.
     */
    uint16_t available() const;

private:
    /**
     * @brief Metered bytes sent since HELLO, wrapping around.
     */
    uint32_t sent{};

    /**
     * @brief Metered bytes the application last said it consumed since HELLO, wrapping around.
     */
    uint32_t consumed{};
};
}  // namespace micras::comm

#endif  // MICRAS_COMM_CREDIT_WINDOW_HPP
