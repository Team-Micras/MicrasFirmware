/**
 * @file buzzer.hpp
 *
 * @brief Proxy Buzzer class declaration
 *
 * @date 03/2024
 */

#ifndef MICRAS_PROXY_BUZZER_HPP
#define MICRAS_PROXY_BUZZER_HPP

#include <cstdint>

#include "micras/hal/pwm.hpp"
#include "micras/hal/timer.hpp"

namespace micras::proxy {
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
    explicit Buzzer(const Config& config);

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

    /**
     * @brief Wait for a duration updating the buzzer
     *
     * @param duration Duration to wait in ms
     */
    void wait(uint32_t duration);

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
    bool is_playing{};

    /**
     * @brief Duration of the sound
     */
    uint32_t duration{};
};
}  // namespace micras::proxy

#endif  // MICRAS_PROXY_BUZZER_HPP
