/**
 * @file
 */

#ifndef MICRAS_PROXY_ARGB_HPP
#define MICRAS_PROXY_ARGB_HPP

#include <array>
#include <cstdint>

#include "micras/hal/pwm_dma.hpp"

namespace micras::proxy {
/**
 * @brief Class for controlling an addressable RGB LED.
 */
template <uint8_t num_of_leds>
class TArgb {
public:
    /**
     * @brief Configuration struct for the addressable RGB LED.
     */
    struct Config {
        hal::PwmDma::Config pwm;
        float               low_duty_cycle;
        float               high_duty_cycle;
        float               max_brightness;
    };

    /**
     * @brief Struct for storing color information.
     */
    struct Color {
        uint8_t red;
        uint8_t green;
        uint8_t blue;

        Color operator*(float brightness) const {
            return {
                static_cast<uint8_t>(this->red * brightness),
                static_cast<uint8_t>(this->green * brightness),
                static_cast<uint8_t>(this->blue * brightness),
            };
        }
    };

    /**
     * @brief Predefined colors.
     */
    struct Colors {
        Colors() = delete;

        static constexpr Color red{255, 0, 0};
        static constexpr Color green{0, 255, 0};
        static constexpr Color blue{0, 0, 255};
        static constexpr Color yellow{255, 255, 0};
        static constexpr Color cyan{0, 255, 255};
        static constexpr Color magenta{255, 0, 255};
        static constexpr Color white{255, 255, 255};
    };

    /**
     * @brief Construct a new Argb object.
     *
     * @param config Configuration for the addressable RGB LED.
     */
    explicit TArgb(const Config& config);

    /**
     * @brief Set the color of the ARGB at the specified index.
     *
     * @param index The index of the ARGB to set the color of.
     * @param color The color to set the ARGB to.
     */
    void set_color(const Color& color, uint8_t index);

    /**
     * @brief Set the color of all ARGBs.
     *
     * @param color The color to set all ARGBs to.
     */
    void set_color(const Color& color);

    /**
     * @brief Set the colors of all ARGBs.
     *
     * @param colors The colors to set the ARGBs to.
     */
    void set_colors(const std::array<Color, num_of_leds>& colors);

    /**
     * @brief Turn off the ARGB at the specified index.
     *
     * @param index The index of the ARGB to turn off.
     */
    void turn_off(uint8_t index);

    /**
     * @brief Turn off all ARGBs.
     */
    void turn_off();

    /**
     * @brief Send the colors to the addressable RGB LED.
     *
     * This function is called automatically when the colors are set.
     */
    void update();

private:
    /**
     * @brief Encode a color into the data buffer.
     *
     * @param color The color to encode.
     * @param index The index to encode the color at.
     */
    void encode_color(const Color& color, uint8_t index);

    /**
     * @brief Protocol constants for the addressable RGB LED communication.
     */
    ///@{
    static constexpr uint8_t bits_per_color{8};
    static constexpr uint8_t colors_per_led{3};
    static constexpr uint8_t bits_per_led{bits_per_color * colors_per_led};
    static constexpr uint8_t reset_length{225};
    ///@}

    /**
     * @brief PWM control object.
     */
    hal::PwmDma pwm;

    /**
     * @brief PWM autoreload value for the low signal.
     */
    uint32_t low_bit;

    /**
     * @brief PWM autoreload value for the high signal.
     */
    uint32_t high_bit;

    /**
     * @brief Maximum brightness of the addressable RGB LED.
     */
    float brightness;

    /**
     * @brief Array to store the color of each LED.
     */
    std::array<Color, num_of_leds> colors{};

    /**
     * @brief Data buffer to send to the addressable RGB LED.
     */
    std::array<uint16_t, num_of_leds * bits_per_led + reset_length> buffer{};
};
}  // namespace micras::proxy

#include "../src/argb.cpp"  // NOLINT(bugprone-suspicious-include, misc-header-include-cycle)

#endif  // MICRAS_PROXY_ARGB_HPP
