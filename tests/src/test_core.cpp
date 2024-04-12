/**
 * @file test_core.cpp
 *
 * @brief Core class to the test
 *
 * @date 04/2024
 */

#include "micras/hal/mcu.hpp"
#include "test_core.hpp"

namespace micras {
void TestCore::init(int /*argc*/, char** /*argv*/) {
    hal::Mcu::init();
}

void TestCore::loop(const std::function<void()>& loop_func) {
    while (true) {
        loop_func();
    }
}
}  // namespace micras
