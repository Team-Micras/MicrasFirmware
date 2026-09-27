/**
 * @file
 */

#ifndef MICRAS_PROXY_WATCHDOG_HPP
#define MICRAS_PROXY_WATCHDOG_HPP

#include <cstdint>

namespace micras::proxy {
/**
 * @brief Class for keeping the watchdog of the microcontroller fed.
 *
 * @note Once started the watchdog cannot be stopped, so anything that hangs for longer than its
 * timeout resets the microcontroller, which brings every actuator back to its inactive state.
 */
class Watchdog {
public:
    /**
     * @brief Configuration struct for the watchdog.
     */
    struct Config {
        uint32_t timeout_ms;
    };

    /**
     * @brief Temporary widening of the timeout, which ends with the life of the object.
     *
     * @note Meant for the few operations that legitimately stall the core for longer than the
     * control loop budget, such as erasing a flash sector, and only with the robot stopped.
     */
    class Extension {
    public:
        /**
         * @brief Widen the timeout of a watchdog.
         *
         * @param watchdog The watchdog.
         * @param timeout_ms The timeout while the extension lives, in milliseconds.
         */
        Extension(Watchdog& watchdog, uint32_t timeout_ms);

        /**
         * @brief Bring the timeout back to the configured one.
         */
        ~Extension();

        /**
         * @brief Special member functions deleted, since an extension ends exactly once.
         */
        ///@{
        Extension(const Extension&) = delete;
        Extension(Extension&&) = delete;
        Extension& operator=(const Extension&) = delete;
        Extension& operator=(Extension&&) = delete;
        ///@}

    private:
        /**
         * @brief Watchdog whose timeout was widened.
         */
        // NOLINTNEXTLINE(*-avoid-const-or-ref-data-members) the extension cannot outlive the watchdog
        Watchdog& watchdog;
    };

    /**
     * @brief Construct a new Watchdog object, starting the watchdog.
     *
     * @param config Configuration for the watchdog.
     */
    explicit Watchdog(const Config& config);

    /**
     * @brief Tell the watchdog that the program is alive.
     */
    void refresh();

    /**
     * @brief Widen the timeout for as long as the returned object lives.
     *
     * @param timeout_ms The timeout during the extension, in milliseconds.
     * @return The extension.
     */
    [[nodiscard]] Extension extend(uint32_t timeout_ms);

private:
    /**
     * @brief Timeout outside of any extension, in milliseconds.
     */
    uint32_t timeout_ms;
};
}  // namespace micras::proxy

#endif  // MICRAS_PROXY_WATCHDOG_HPP
