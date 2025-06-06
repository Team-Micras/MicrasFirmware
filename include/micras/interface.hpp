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
#include "micras/comm/serial_variable_pool.hpp"

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
     * @param argb Shared pointer to the addressable RGB LED object.
     * @param button Shared pointer to the button object.
     * @param buzzer Shared pointer to the buzzer object.
     * @param dip_switch Shared pointer to the DIP switch object.
     * @param led Shared pointer to the LED object.
     */
    Interface(
        const std::shared_ptr<comm::SerialVariablePool>& pool, const std::shared_ptr<proxy::TArgb<2>>& argb,
        const std::shared_ptr<proxy::Button>& button, const std::shared_ptr<proxy::Buzzer>& buzzer,
        const std::shared_ptr<proxy::TDipSwitch<4>>& dip_switch, const std::shared_ptr<proxy::Led>& led
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
     * @brief Enum for the type of condition to check.
     */
    enum class ConditionType {
        Trigger,
        Switch
    };

    /**
     * @brief Type for the condition function pointer.
     */
    using ConditionFunc = bool (Interface::*)() const;

    struct EventCondition {
        Event         true_event;
        Event         false_event;
        ConditionType type;
        ConditionFunc check;
    };

    /**
     * @brief Enum for what each dip switch pin does.
     */
    enum DipSwitchPins : uint8_t {
        FAN = 0,
        DIAGONAL = 1,
        BOOST = 2,
        RISKY = 3,
    };

    /**
     * @brief Array of event conditions for each event.
     */
    std::array<EventCondition, NUMBER_OF_EVENTS> event_conditions = {
        {{
             .true_event = Event::EXPLORE,
             .type = ConditionType::Trigger,
             .check = &Interface::condition_explore,
         },
         {
             .true_event = Event::SOLVE,
             .type = ConditionType::Trigger,
             .check = &Interface::condition_solve,
         },
         {
             .true_event = Event::CALIBRATE,
             .type = ConditionType::Trigger,
             .check = &Interface::condition_calibrate,
         },
         {
             .true_event = Event::TURN_ON_FAN,
             .false_event = Event::TURN_OFF_FAN,
             .type = ConditionType::Switch,
             .check = &Interface::condition_fan,
         },
         {
             .true_event = Event::TURN_ON_DIAGONAL,
             .false_event = Event::TURN_OFF_DIAGONAL,
             .type = ConditionType::Switch,
             .check = &Interface::condition_diagonal,
         },
         {
             .true_event = Event::TURN_ON_BOOST,
             .false_event = Event::TURN_OFF_BOOST,
             .type = ConditionType::Switch,
             .check = &Interface::condition_boost,
         },
         {
             .true_event = Event::TURN_ON_RISKY,
             .false_event = Event::TURN_OFF_RISKY,
             .type = ConditionType::Switch,
             .check = &Interface::condition_risky,
         }}
    };

    /**
     * @brief Methods for checking the conditions for each event.
     *
     * @return true if the condition is met, false otherwise.
     */
    ///@{
    bool condition_explore() const;
    bool condition_solve() const;
    bool condition_calibrate() const;
    bool condition_fan() const;
    bool condition_diagonal() const;
    bool condition_boost() const;
    bool condition_risky() const;

    ///@}

    /**
     * @brief Serial variable pool for managing variables.
     */
    std::shared_ptr<comm::SerialVariablePool> pool;

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

    /**
     * @brief Array to store the last dip switch states.
     */
    std::array<bool, 4> dip_switch_states{};

    /**
     * @brief Variables synced with the serial variable pool.
     */
    ///@{
    bool comm_explore{false};
    bool comm_solve{false};
    bool comm_calibrate{false};
    bool comm_fan{false};
    bool comm_diagonal{false};
    bool comm_boost{false};
    bool comm_risky{false};
    ///@}
};
}  // namespace micras

#endif  // MICRAS_INTERFACE_HPP
