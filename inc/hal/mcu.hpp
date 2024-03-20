/**
 * @file mcu.hpp
 *
 * @brief MCU related
 */

#ifndef __MCU_HPP__
#define __MCU_HPP__

#include <stdint.h>

/*****************************************
 * Public Function Prototypes
 *****************************************/

extern "C"
{
/**
 * @brief Initializes System Clock
 * @note  Defined by cube
 */
void SystemClock_Config(void);
}

namespace hal {
class mcu {
    public:
        /**
         * @brief Initializes MCU and some peripherals
         */
        static void init(void);

        /**
         * @brief Put the MCU to sleep
         *
         * @param ms Sleep time in milliseconds
         */
        static void sleep(uint32_t ms);

        /**
         * @brief Reset timer.
         *
         * @param tim_counter Timer counter to reset.
         */
        static void reset_timer(uint32_t& tim_counter);

        /**
         * @brief Get timer value in milliseconds.
         *
         * @param tim_counter Timer counter to get value from.
         * @return Timer value in milliseconds.
         */
        static uint32_t get_timer_ms(uint32_t tim_counter);
};
};
#endif // __MCU_HPP__
