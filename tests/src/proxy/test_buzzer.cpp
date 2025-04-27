/**
 * @file
 */

#include <array>

#include "test_core.hpp"

using namespace micras;  // NOLINT(google-build-using-namespace)

static constexpr std::array<uint32_t, 39> frequencies{
    987,  987,  987,  987,  987,  1046, 1318, 987,  987,  987, 1046, 1318, 987, 987, 987, 1046, 1318, 1479, 1479, 1760,
    1567, 1479, 1318, 1318, 1760, 1567, 1479, 1318, 1318, 987, 1046, 1318, 987, 987, 987, 1046, 1318, 987,  987,
};
static constexpr std::array<uint32_t, frequencies.size()> intervals{
    333, 333, 333, 333, 250, 250, 167, 333, 333, 250, 250, 167, 333, 333, 250, 250, 167, 333, 333, 250,
    250, 167, 333, 333, 250, 250, 167, 333, 333, 250, 250, 167, 333, 333, 250, 250, 167, 333, 0,
};
static constexpr uint32_t duration{133};

int main(int argc, char* argv[]) {
    TestCore::init(argc, argv);
    proxy::Button button{button_config};
    proxy::Buzzer buzzer{buzzer_config};

    TestCore::loop([&button, &buzzer]() {
        while (button.get_status() == proxy::Button::Status::NO_PRESS) {
            button.update();
        }

        for (uint32_t i = 0; i < frequencies.size(); i++) {
            buzzer.play(frequencies.at(i), duration);
            buzzer.wait(intervals.at(i));
        }
    });

    return 0;
}
