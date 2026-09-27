/**
 * @file
 */

#ifndef MICRAS_CLIP_PLAYER_HPP
#define MICRAS_CLIP_PLAYER_HPP

#include <array>
#include <cstdint>

#include "micras/hal/pwm.hpp"
#include "target.hpp"

namespace micras {
/**
 * @brief Player of an audio clip through the buzzer, for the tests that play sounds.
 *
 * @details The buzzer PWM becomes an 8-bit DAC: TIM15 runs without prescaler at three times the
 * sample rate, and its update interrupt moves the duty cycle linearly from each sample to the next,
 * which keeps the images of the audio away from the band the buzzer reproduces. The clips come from
 * scripts/buzzer_audio.py, and the test forwards TIM15_IRQHandler to update().
 *
 * @tparam samples Samples of the clip, as duty cycles of the buzzer from 0 to 255.
 * @tparam sample_rate Rate of the samples in Hz.
 */
template <const auto& samples, uint32_t sample_rate>
class ClipPlayer {
public:
    ClipPlayer() = delete;

    /**
     * @brief Set the buzzer timer up for the clip and enable its interrupt.
     */
    static void init() {
        hal::Pwm pwm{buzzer_config.pwm};

        TIM15->PSC = 0;
        pwm.set_frequency(sample_rate * interpolation);
        TIM15->EGR = TIM_EGR_UG;
        TIM15->SR = ~TIM_SR_UIF;

        const uint32_t period = TIM15->ARR + 1;

        for (uint32_t code = 0; code < compare_levels.size(); code++) {
            compare_levels.at(code) = (code * period + 127) / 255;
        }

        HAL_NVIC_SetPriority(TIM15_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(TIM15_IRQn);
    }

    /**
     * @brief Play the clip from its start.
     */
    static void play() {
        sample_index = 0;
        interpolation_step = 0;
        fade_left = fade_periods;
        stopping = false;
        playing = true;
        TIM15->SR = ~TIM_SR_UIF;
        TIM15->DIER |= TIM_DIER_UIE;
    }

    /**
     * @brief Fade the buzzer out and stop the clip.
     */
    static void stop() { stopping = true; }

    /**
     * @brief Check whether the clip is playing.
     *
     * @return True while the clip plays or fades out, false otherwise.
     */
    static bool is_playing() { return playing; }

    /**
     * @brief Set the duty cycle of the next carrier period from the samples.
     */
    static void update() {
        TIM15->SR = ~TIM_SR_UIF;

        const uint32_t index = sample_index;
        uint32_t       fade = fade_left;

        if (stopping) {
            fade = fade > 0 ? fade - 1 : 0;
            fade_left = fade;
        }

        if (fade == 0 or index + 1 >= samples.size()) {
            TIM15->CCR1 = 0;
            TIM15->DIER &= ~TIM_DIER_UIE;
            playing = false;
            return;
        }

        const uint32_t step = interpolation_step;
        const uint32_t current = compare_levels.at(samples.at(index));
        const uint32_t next = compare_levels.at(samples.at(index + 1));
        const uint32_t blended = current * (interpolation - step) + next * step;

        TIM15->CCR1 = blended * fade / (interpolation * fade_periods);

        if (step + 1 == interpolation) {
            interpolation_step = 0;
            sample_index = index + 1;
        } else {
            interpolation_step = step + 1;
        }
    }

private:
    /**
     * @brief Carrier periods per sample, over which the duty cycle moves linearly to the next sample.
     */
    static constexpr uint32_t interpolation{3};

    /**
     * @brief Carrier periods over which a stop fades the buzzer out.
     */
    static constexpr uint32_t fade_periods{sample_rate * interpolation / 8};

    // NOLINTBEGIN(cppcoreguidelines-avoid-non-const-global-variables)

    /**
     * @brief Compare value of each sample code, for the period of the carrier.
     */
    inline static std::array<uint32_t, 256> compare_levels{};

    /**
     * @brief Index of the sample the duty cycle is moving from.
     */
    inline static volatile uint32_t sample_index{};

    /**
     * @brief Carrier period within the current sample.
     */
    inline static volatile uint32_t interpolation_step{};

    /**
     * @brief Carrier periods left in the fade out.
     */
    inline static volatile uint32_t fade_left{};

    /**
     * @brief Whether a stop was requested.
     */
    inline static volatile bool stopping{};

    /**
     * @brief Whether the clip is playing.
     */
    inline static volatile bool playing{};

    // NOLINTEND(cppcoreguidelines-avoid-non-const-global-variables)
};
}  // namespace micras

#endif  // MICRAS_CLIP_PLAYER_HPP
