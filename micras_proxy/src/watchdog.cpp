/**
 * @file
 */

#include <cstdint>

#include "micras/hal/mcu.hpp"
#include "micras/proxy/watchdog.hpp"

namespace micras::proxy {
Watchdog::Extension::Extension(Watchdog& watchdog, uint32_t timeout_ms) : watchdog{watchdog} {
    hal::Mcu::set_watchdog_timeout(timeout_ms);
}

Watchdog::Extension::~Extension() {
    hal::Mcu::set_watchdog_timeout(this->watchdog.timeout_ms);
}

Watchdog::Watchdog(const Config& config) : timeout_ms{config.timeout_ms} {
    hal::Mcu::set_watchdog_timeout(this->timeout_ms);
}

// NOLINTNEXTLINE(readability-convert-member-functions-to-static) there is one watchdog, and this object is it
void Watchdog::refresh() {
    hal::Mcu::refresh_watchdog();
}

Watchdog::Extension Watchdog::extend(uint32_t timeout_ms) {
    return Extension{*this, timeout_ms};
}
}  // namespace micras::proxy
