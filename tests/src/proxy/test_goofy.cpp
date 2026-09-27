/**
 * @file
 */

#include "clip_player.hpp"
#include "clips/goofy.hpp"
#include "micras/proxy/button.hpp"
#include "target.hpp"
#include "test_core.hpp"

using namespace micras;  // NOLINT(google-build-using-namespace)

using Player = ClipPlayer<clips::goofy::samples, clips::goofy::sample_rate>;

/**
 * @brief Forward the buzzer timer interrupt to the player.
 */
extern "C" void TIM15_IRQHandler() {
    Player::update();
}

/**
 * @brief Play a medley of goofy cartoon sound effects through the buzzer, started and stopped by the button.
 */
int main(int argc, char* argv[]) {
    TestCore::init(argc, argv);
    proxy::Button button{button_config};
    Player::init();

    TestCore::loop([&button]() {
        button.update();

        if (button.get_status() == proxy::Button::Status::NO_PRESS) {
            return;
        }

        if (Player::is_playing()) {
            Player::stop();
        } else {
            Player::play();
        }
    });

    return 0;
}
