#ifndef MICRAS_COMM_LOGGER_HPP
#define MICRAS_COMM_LOGGER_HPP

#include <string>
#include <queue>

namespace micras::comm {
class Logger {
public:
    explicit Logger(bool enabled = true);

    void log(const std::string& message);

    std::string get_next_log();

    bool has_logs() const;

    void clear_logs();

    bool is_enabled() const;

private:
    bool enabled;

    std::queue<std::string> log_queue;

};
}

#endif // MICRAS_COMM_LOGGER_HPP
