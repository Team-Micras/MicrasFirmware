#ifndef __MICRAS_CONTROLLER_TEST_HPP__
#define __MICRAS_CONTROLLER_TEST_HPP__

#include "proxy/button.hpp"
#include "proxy/led.hpp"

void micras_controller_test_loop(proxy::Button& button, proxy::Led& led);

#endif // __MICRAS_CONTROLLER_TEST_HPP__
