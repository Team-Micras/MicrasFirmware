/**
 * @file
 */

#ifndef MICRAS_INTERFACE_HPP
#define MICRAS_INTERFACE_HPP

#include <memory>

#include "micras/proxy/argb.hpp"
#include "micras/proxy/button.hpp"
#include "micras/proxy/buzzer.hpp"
#include "micras/proxy/dip_switch.hpp"
#include "micras/proxy/led.hpp"

namespace micras {
/**
 * @brief Class for controlling the robot interface with the external world.
 */
class Interface {
public:
    /**
     * @brief Enum for the events that can be sent to the interface.
     */
    enum Event : uint8_t {
        EXPLORE = 0,
        SOLVE = 1,
        CALIBRATE = 2,
        ERROR = 3,
        NUMBER_OF_EVENTS = 4,
    };

    /**
     * @brief Construct a new Interface object.
     *
     * @param argb Shared pointer to the addressable RGB LED object.
     * @param button Shared pointer to the button object.
     * @param buzzer Shared pointer to the buzzer object.
     * @param dip_switch Shared pointer to the DIP switch object.
     * @param led Shared pointer to the LED object.
     */
    Interface(
        const std::shared_ptr<proxy::TArgb<2>>& argb, const std::shared_ptr<proxy::Button>& button,
        const std::shared_ptr<proxy::Buzzer>& buzzer, const std::shared_ptr<proxy::TDipSwitch<4>>& dip_switch,
        const std::shared_ptr<proxy::Led>& led
    );

    /**
     * @brief Update the interface.
     */
    void update();

    /**
     * @brief Send an event to the interface.
     *
     * @param event The event to send.
     */
    void send_event(Event event);

    /**
     * @brief Get the value of an event and reset it.
     *
     * @param event The event to get.
     * @return True if the event happened, false otherwise.
     */
    bool acknowledge_event(Event event);

    /**
     * @brief Get the value of an event without reseting it.
     *
     * @param event The event to get.
     * @return True if the event happened, false otherwise.
     */
    bool peek_event(Event event) const;

private:
    /**
     * @brief Addressable RGB LED object.
     */
    std::shared_ptr<proxy::TArgb<2>> argb;

    /**
     * @brief Button object.
     */
    std::shared_ptr<proxy::Button> button;

    /**
     * @brief Buzzer object.
     */
    std::shared_ptr<proxy::Buzzer> buzzer;

    /**
     * @brief Dip switch object.
     */
    std::shared_ptr<proxy::TDipSwitch<4>> dip_switch;

    /**
     * @brief LED object.
     */
    std::shared_ptr<proxy::Led> led;

    /**
     * @brief Array of the listed events.
     */
    std::array<bool, Event::NUMBER_OF_EVENTS> events{};
};
}  // namespace micras

#endif  // MICRAS_INTERFACE_HPP
