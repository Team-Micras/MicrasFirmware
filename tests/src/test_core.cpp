/**
 * @file test_core.cpp
 *
 * @brief Core functions to the test
 *
 * @date 04/2024
 */

#include "hal/mcu.hpp"
#include "test_core.hpp"

void test_core_init() {
    hal::mcu::init();
}
