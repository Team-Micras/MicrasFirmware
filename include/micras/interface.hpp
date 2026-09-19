/**
 * @file
 */

#ifndef MICRAS_INTERFACE_HPP
#define MICRAS_INTERFACE_HPP

#include <array>
#include <cstdint>
#include <utility>

#include "micras/proxy/button.hpp"
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
    enum class Event : uint8_t {
        EXPLORE = 0,
        SOLVE = 1,
        CALIBRATE = 2,
        ERROR = 3,
        TURN_ON_FAN = 4,
        TURN_OFF_FAN = 5,
        TURN_ON_DIAGONAL = 6,
        TURN_OFF_DIAGONAL = 7,
        TURN_ON_BOOST = 8,
        TURN_OFF_BOOST = 9,
        TURN_ON_RISKY = 10,
        TURN_OFF_RISKY = 11,
        NUMBER_OF_EVENTS = 12,
    };

    /**
     * @brief Construct a new Interface object.
     *
     * @note The proxies are borrowed, not owned: they live in the Micras object for the whole
     * program, which is why they are references and not pointers of any kind.
     *
     * @param button The button object.
     * @param dip_switch The DIP switch object.
     * @param led The LED object.
     */
    Interface(const proxy::Button& button, const proxy::TDipSwitch<4>& dip_switch, proxy::Led& led);

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
     * @brief Enum for what each dip switch pin does.
     */
    enum class DipSwitchPins : uint8_t {
        FAN = 0,
        DIAGONAL = 1,
        BOOST = 2,
        RISKY = 3,
    };

    // NOLINTBEGIN(*-avoid-const-or-ref-data-members) borrowed for the lifetime of the robot
    /**
     * @brief Button object.
     */
    const proxy::Button& button;

    /**
     * @brief Dip switch object.
     */
    const proxy::TDipSwitch<4>& dip_switch;

    /**
     * @brief LED object.
     */
    proxy::Led& led;
    // NOLINTEND(*-avoid-const-or-ref-data-members)

    /**
     * @brief Array of the listed events.
     */
    std::array<bool, std::to_underlying(Event::NUMBER_OF_EVENTS)> events{};

    /**
     * @brief Array to store the last dip switch states.
     */
    std::array<bool, 4> dip_switch_states{};
};
}  // namespace micras

#endif  // MICRAS_INTERFACE_HPP
