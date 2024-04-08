/**
 * @file test_core.cpp
 *
 * @brief Core functions to the test
 *
 * @date 04/2024
 */

#include "micras/hal/mcu.hpp"
#include "test_core.hpp"

namespace micras {
void test_core_init() {
    hal::Mcu::init();
}
}  // namespace micras
