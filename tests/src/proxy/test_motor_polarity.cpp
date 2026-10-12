/**
 * @file
 */

#include "micras/proxy/button.hpp"
#include "micras/proxy/locomotion.hpp"
#include "target.hpp"
#include "test_core.hpp"

using namespace micras;  // NOLINT(google-build-using-namespace)

// NOLINTBEGIN(cppcoreguidelines-avoid-non-const-global-variables)
static volatile float test_right_command{15.0F};
static volatile bool  test_spinning{};

// NOLINTEND(cppcoreguidelines-avoid-non-const-global-variables)

int main(int argc, char* argv[]) {
    TestCore::init(argc, argv);
    proxy::Button     button{button_config};
    proxy::Locomotion locomotion{locomotion_config};

    TestCore::loop([&button, &locomotion]() {
        button.update();

        if (button.get_status() == proxy::Button::Status::SHORT_PRESS) {
            test_spinning = not test_spinning;

            if (test_spinning) {
                locomotion.enable();
            } else {
                locomotion.stop();
                locomotion.disable();
            }
        }

        if (test_spinning) {
            locomotion.set_wheel_command(0.0F, test_right_command);
        }
    });

    return 0;
}
