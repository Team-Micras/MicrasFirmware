/**
 * @file buzzer.hpp
 *
 * @brief Proxy Buzzer class declaration
 *
 * @date 03/2024
 */

#ifndef __BUZZER_HPP__
#define __BUZZER_HPP__

#include <cstdint>

#include "hal/pwm.hpp"
#include "hal/timer.hpp"

namespace proxy {
/**
 * @brief Class for controlling a buzzer
 */
class Buzzer {
    public:
        /**
         * @brief Configuration structure for the buzzer
         */
        struct Config {
            hal::Pwm::Config pwm;
        };

        /**
         * @brief Constructor for the Buzzer class
         *
         * @param config Configuration for the buzzer
         */
        Buzzer(Config& config);

        /**
         * @brief Play a tone for a duration
         *
         * @param frequency Buzzer sound frequency in Hz
         * @param duration Duration of the sound in ms
         */
        void play(uint32_t frequency, uint32_t duration = 0);

        /**
         * @brief Update the buzzer state
         */
        void update();

        /**
         * @brief Stop the buzzer sound
         */
        void stop();

    private:
        /**
         * @brief PWM object
         */
        hal::Pwm pwm;

        /**
         * @brief Timer to play the sound
         */
        hal::Timer timer;

        /**
         * @brief Flag to check if the buzzer is playing
         */
        bool is_playing{false};

        /**
         * @brief Duration of the sound
         */
        uint32_t duration{0};
};
}  // namespace proxy

#endif // __BUZZER_HPP__
