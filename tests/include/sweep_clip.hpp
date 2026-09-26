/**
 * @file
 */

#ifndef MICRAS_SWEEP_CLIP_HPP
#define MICRAS_SWEEP_CLIP_HPP

#include <array>
#include <cmath>
#include <cstdint>
#include <numbers>

/**
 * @brief Clip that test_chatuba plays until scripts/buzzer_audio.py writes the song over it.
 *
 * @details A logarithmic sine sweep from 100 Hz to 9 kHz, between two ramps of the duty cycle from
 * zero to the middle of its range and back, the same framing the script gives the song. Listening to
 * it checks the audio path and the frequency response of the buzzer.
 */
namespace micras::audio_clip {
/**
 * @brief Rate of the samples in Hz.
 */
inline constexpr uint32_t sample_rate{20000};

/**
 * @brief Duration of each ramp in samples.
 */
inline constexpr uint32_t ramp_samples{sample_rate / 4};

/**
 * @brief Duration of the sweep in samples.
 */
inline constexpr uint32_t sweep_samples{sample_rate * 6};

/**
 * @brief Sweep samples, as duty cycles from 0 to 255.
 */
inline const std::array<uint8_t, sweep_samples + 2 * ramp_samples> samples = [] {
    constexpr float start_frequency{100.0F};
    constexpr float end_frequency{9000.0F};
    constexpr float middle{127.5F};

    const float duration = static_cast<float>(sweep_samples) / sample_rate;
    const float growth = std::log(end_frequency / start_frequency);

    std::array<uint8_t, sweep_samples + 2 * ramp_samples> result{};

    for (uint32_t i = 0; i < ramp_samples; i++) {
        const float position = static_cast<float>(i) / ramp_samples;
        const float level = position * position * (3.0F - 2.0F * position);
        result.at(i) = static_cast<uint8_t>(std::lround(middle * level));
        result.at(result.size() - 1 - i) = result.at(i);
    }

    for (uint32_t i = 0; i < sweep_samples; i++) {
        const float time = static_cast<float>(i) / sample_rate;
        const float phase = 2.0F * std::numbers::pi_v<float> * start_frequency * duration / growth *
                            (std::exp(growth * time / duration) - 1.0F);
        result.at(ramp_samples + i) = static_cast<uint8_t>(std::lround(middle + middle * std::sin(phase)));
    }

    return result;
}();
}  // namespace micras::audio_clip

#endif  // MICRAS_SWEEP_CLIP_HPP
