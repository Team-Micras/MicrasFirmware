/**
 * @file
 */

#include <array>
#include <cstdint>

#include "chatuba.hpp"
#include "micras/hal/pwm.hpp"
#include "micras/proxy/button.hpp"
#include "target.hpp"
#include "test_core.hpp"

using namespace micras;  // NOLINT(google-build-using-namespace)

/**
 * @brief Carrier periods per sample, over which the duty cycle moves linearly to the next sample.
 */
static constexpr uint32_t interpolation{3};

/**
 * @brief Carrier periods over which a stop fades the buzzer out.
 */
static constexpr uint32_t fade_periods{audio_clip::sample_rate * interpolation / 8};

// NOLINTBEGIN(cppcoreguidelines-avoid-non-const-global-variables)
static std::array<uint32_t, 256> compare_levels{};
static volatile uint32_t         sample_index{};
static volatile uint32_t         interpolation_step{};
static volatile uint32_t         fade_left{};
static volatile bool             stopping{};
static volatile bool             playing{};

// NOLINTEND(cppcoreguidelines-avoid-non-const-global-variables)

/**
 * @brief Set the duty cycle of the next carrier period from the samples.
 */
extern "C" void TIM15_IRQHandler() {
    TIM15->SR = ~TIM_SR_UIF;

    const uint32_t index = sample_index;
    uint32_t       fade = fade_left;

    if (stopping) {
        fade = fade > 0 ? fade - 1 : 0;
        fade_left = fade;
    }

    if (fade == 0 or index + 1 >= audio_clip::samples.size()) {
        TIM15->CCR1 = 0;
        TIM15->DIER &= ~TIM_DIER_UIE;
        playing = false;
        return;
    }

    const uint32_t step = interpolation_step;
    const uint32_t current = compare_levels.at(audio_clip::samples.at(index));
    const uint32_t next = compare_levels.at(audio_clip::samples.at(index + 1));
    const uint32_t blended = current * (interpolation - step) + next * step;

    TIM15->CCR1 = blended * fade / (interpolation * fade_periods);

    if (step + 1 == interpolation) {
        interpolation_step = 0;
        sample_index = index + 1;
    } else {
        interpolation_step = step + 1;
    }
}

/**
 * @brief Play the clip from its start.
 */
static void start_playback() {
    sample_index = 0;
    interpolation_step = 0;
    fade_left = fade_periods;
    stopping = false;
    playing = true;
    TIM15->SR = ~TIM_SR_UIF;
    TIM15->DIER |= TIM_DIER_UIE;
}

/**
 * @brief Play the clip in chatuba.hpp through the buzzer, started and stopped by the button.
 *
 * @details The buzzer PWM becomes an 8-bit DAC: TIM15 runs without prescaler at three times the
 * sample rate, and its update interrupt moves the duty cycle linearly from each sample to the next,
 * which keeps the images of the audio away from the band the buzzer reproduces. A press during
 * playback fades the buzzer out.
 */
int main(int argc, char* argv[]) {
    TestCore::init(argc, argv);
    proxy::Button button{button_config};
    hal::Pwm      pwm{buzzer_config.pwm};

    TIM15->PSC = 0;
    pwm.set_frequency(audio_clip::sample_rate * interpolation);
    TIM15->EGR = TIM_EGR_UG;
    TIM15->SR = ~TIM_SR_UIF;

    const uint32_t period = TIM15->ARR + 1;

    for (uint32_t code = 0; code < compare_levels.size(); code++) {
        compare_levels.at(code) = (code * period + 127) / 255;
    }

    HAL_NVIC_SetPriority(TIM15_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM15_IRQn);

    TestCore::loop([&button]() {
        button.update();

        if (button.get_status() == proxy::Button::Status::NO_PRESS) {
            return;
        }

        if (playing) {
            stopping = true;
        } else {
            start_playback();
        }
    });

    return 0;
}
