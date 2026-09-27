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
        PROFILE_MOVED = 4,
        NUMBER_OF_EVENTS = 5,
    };

    /**
     * @brief Bits of the run profile, one per DIP switch.
     */
    enum class Profile : uint8_t {
        FAN = 1U << 0U,
        RACING_LINE = 1U << 1U,
        BOOST = 1U << 2U,
        RISKY = 1U << 3U,
    };

    /**
     * @brief Construct a new Interface object.
     *
     * @note The proxies are borrowed, not owned: they live in the Micras object for the whole
     * program, so they are taken by reference.
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

    /**
     * @brief Get the run profile the switches currently spell out.
     *
     * @note The switches are one of two writers of the run profile, the link being the other, so
     * this is read when they move rather than being consulted every time the profile matters.
     *
     * @return One bit per switch, in the order of the Profile enum.
     */
    uint8_t get_profile() const;

private:
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
     * @brief Last read state of the switches, as a run profile.
     */
    uint8_t profile{};
};
}  // namespace micras

#endif  // MICRAS_INTERFACE_HPP
