#include "micras/comm/logger.hpp"

namespace micras::comm {
    Logger::Logger(bool enabled) : enabled{enabled} { }

    void Logger::log(const std::string& message) {
        if (this->enabled) {
            log_queue.push(message);
        }
    }

    std::string Logger::get_next_log() {
        if (log_queue.empty()) {
            return "";
        }
        std::string log = log_queue.front();
        log_queue.pop();
        return log;
    }

    bool Logger::has_logs() const {
        return !log_queue.empty();
    }

    void Logger::clear_logs() {
        while (!log_queue.empty()) {
            log_queue.pop();
        }
    }

    bool Logger::is_enabled() const {
        return enabled;
    }
}
