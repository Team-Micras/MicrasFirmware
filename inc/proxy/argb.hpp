/**
 * @file argb.hpp
 *
 * @brief Proxy Argb class declaration
 *
 * @date 03/2024
 */

#ifndef __ARGB_HPP__
#define __ARGB_HPP__

#include <array>
#include <cstdint>

#include "hal/pwm_dma.hpp"

namespace proxy {
/**
 * @brief Class for controlling an addressable RGB LED
 */
template <uint8_t num_of_leds>
class Argb {
    public:
        /**
         * @brief Configuration structure for the addressable RGB LED
         */
        struct Config {
            hal::PwmDma::Config pwm;
        };

        /**
         * @brief Structure for storing color information
         */
        struct Color {
            uint8_t red;
            uint8_t green;
            uint8_t blue;
        };

        /**
         * @brief Constructor for the Argb class
         *
         * @param config Configuration for the addressable RGB LED
         */
        Argb(Config& config);

        /**
         * @brief Set the color of the ARGB at the specified index
         *
         * @param index The index of the ARGB to set the color of
         * @param color The color to set the ARGB to
         */
        void set_color(const Color& color, uint8_t index);

        /**
         * @brief Set the color of all ARGBs
         *
         * @param color The color to set all ARGBs to
         */
        void set_color(const Color& color);

        /**
         * @brief Turn off the ARGB at the specified index
         *
         * @param index The index of the ARGB to turn off
         */
        void turn_off(uint8_t index);

        /**
         * @brief Turn off all ARGBs
         */
        void turn_off();

    private:
        /**
         * @brief Encode a color into the data buffer
         *
         * @param color The color to encode
         * @param index The index to encode the color at
         */
        void encode_color(const Color& color, uint8_t index);

        /**
         * @brief PWM control object
         */
        hal::PwmDma pwm;

        /**
         * @brief PWM autoreload value for the low signal
         */
        const uint32_t low_bit;

        /**
         * @brief PWM autoreload value for the high signal
         */
        const uint32_t high_bit;

        /**
         * @brief Protocol constants for the addressable RGB LED communication
         */
        static constexpr uint8_t bits_per_color{8};
        static constexpr uint8_t colors_per_led{3};
        static constexpr uint8_t bits_per_led{bits_per_color * colors_per_led};
        static constexpr float low_duty_cycle{0.32f};
        static constexpr float high_duty_cycle{0.64f};
        static constexpr uint8_t reset_length{40};

        /**
         * @brief Data buffer to send to the addressable RGB LED
         */
        std::array<uint32_t, num_of_leds * bits_per_led + reset_length> buffer;
};
}  // namespace proxy

#include "../src/proxy/argb.cpp"

#endif // __ARGB_HPP__