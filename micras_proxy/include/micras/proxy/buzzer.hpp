/**
 * @file
 */

#ifndef MICRAS_PROXY_BUZZER_HPP
#define MICRAS_PROXY_BUZZER_HPP

#include <cstdint>

#include "micras/hal/pwm.hpp"
#include "micras/proxy/stopwatch.hpp"

namespace micras::proxy {
/**
 * @brief Class for controlling a buzzer.
 */
class Buzzer {
public:
    /**
     * @brief Configuration struct for the buzzer.
     */
    struct Config {
        hal::Pwm::Config pwm;
    };

    /**
     * @brief Construct a new Buzzer object.
     *
     * @param config Configuration for the buzzer.
     */
    explicit Buzzer(const Config& config);

    /**
     * @brief Play a tone for a duration.
     *
     * @param frequency Buzzer sound frequency in Hz.
     * @param duration Duration of the sound in ms.
     */
    void play(uint32_t frequency, uint32_t duration = 0);

    /**
     * @brief Update the buzzer state.
     */
    void update();

    /**
     * @brief Stop the buzzer sound.
     */
    void stop();

    /**
     * @brief Wait for a time interval updating the buzzer.
     *
     * @param interval Time to wait in ms.
     */
    void wait(uint32_t interval);

private:
    /**
     * @brief PWM object.
     */
    hal::Pwm pwm;

    /**
     * @brief Stopwatch to play the sound.
     */
    proxy::Stopwatch stopwatch;

    /**
     * @brief Flag to check if the buzzer is playing.
     */
    bool is_playing{};

    /**
     * @brief Duration of the sound.
     */
    uint32_t duration{};
};
}  // namespace micras::proxy

#endif  // MICRAS_PROXY_BUZZER_HPP
