#ifndef MICRAS_COMM_LOGGER_HPP
#define MICRAS_COMM_LOGGER_HPP

#include <queue>
#include <string>

namespace micras::comm {
/**
 * @brief Class for logging messages.
 *
 * @note The logger is used to log messages for debugging purposes. It stores
 *       the messages in a queue and provides methods to retrieve and clear
 *       the logs.
 */
class Logger {
public:
    /**
     * @brief Construct a new Logger object.
     *
     * @param enabled Flag to enable or disable logging.
     */
    explicit Logger(bool enabled = true);

    /**
     * @brief Log a message.
     *
     * @param message The message to log.
     */
    void log(const std::string& message);

    /**
     * @brief Consume the next log message.
     *
     * @return The next log message.
     */
    std::string get_next_log();

    /**
     * @brief Check if there are any log messages available.
     *
     * @return true if there are log messages, false otherwise.
     */
    bool has_logs() const;

    /**
     * @brief Clear all log messages.
     */
    void clear_logs();

    /**
     * @brief Check if logging is enabled.
     *
     * @return true if logging is enabled, false otherwise.
     */
    bool is_enabled() const;

private:
    /**
     * @brief Flag to enable or disable logging.
     */
    bool enabled;

    /**
     * @brief Queue to store log messages.
     */
    std::queue<std::string> log_queue;
};
}  // namespace micras::comm

#endif  // MICRAS_COMM_LOGGER_HPP
