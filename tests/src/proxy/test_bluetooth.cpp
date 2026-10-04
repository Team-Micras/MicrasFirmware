/**
 * @file
 */

#include <array>
#include <bit>
#include <charconv>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <system_error>

#include "constants.hpp"
#include "micras/proxy/bluetooth_serial.hpp"
#include "micras/proxy/button.hpp"
#include "micras/proxy/buzzer.hpp"
#include "micras/proxy/led.hpp"
#include "micras/proxy/stopwatch.hpp"
#include "target.hpp"
#include "test_core.hpp"
#include "usart.h"

#if __has_include("bluetooth_pin.hpp")
    #include "bluetooth_pin.hpp"

static_assert(micras::bluetooth_pin.size() == 6, "The module takes a PIN of six digits");
#endif

using namespace micras;  // NOLINT(google-build-using-namespace)

static constexpr std::array<uint32_t, 9> baud_rates{9600, 115200, 230400, 57600, 38400, 19200, 4800, 2400, 1200};

static constexpr std::array<std::string_view, 30> queries{
    "VERS", "VERR", "ADDR", "NAME", "ROLE", "MODE", "TYPE", "PASS", "NOTI", "NOTP",
    "BAUD", "PARI", "STOP", "UUID", "CHAR", "FFE2", "RESP", "COMI", "COMA", "COLA",
    "COSU", "COUP", "ADTY", "ADVI", "POWE", "PWRM", "IMME", "PIO1", "RELI", "SYSK",
};

static constexpr std::array<std::string_view, 2> module_events{"OK+CONN", "OK+LOST"};

namespace {
struct Setting {
    std::string_view key;
    std::string_view command;
};
}  // namespace

static constexpr std::array<Setting, 7> settings{{
    {.key = "role", .command = "AT+ROLE0"},
    {.key = "name", .command = "AT+NAMEMicras"},
    {.key = "mode", .command = "AT+MODE0"},
    {.key = "noti", .command = "AT+NOTI0"},
    {.key = "comi", .command = "AT+COMI0"},
    {.key = "coma", .command = "AT+COMA0"},
    {.key = "baud", .command = "AT+BAUD7"},
}};

static constexpr uint32_t configured_baud_rate{115200};

static constexpr uint32_t    module_boot_time_ms{1000};
static constexpr uint32_t    module_restart_time_ms{2000};
static constexpr uint32_t    reply_timeout_ms{600};
static constexpr uint32_t    reply_quiet_ms{100};
static constexpr uint32_t    probe_rounds{3};
static constexpr uint32_t    byte_time_ms{10};
static constexpr uint32_t    heartbeat_interval_ms{1000};
static constexpr std::size_t max_line_size{256};
static constexpr uint32_t    max_burst_count{10000};
static constexpr std::size_t max_burst_size{200};

// NOLINTBEGIN(cppcoreguidelines-avoid-non-const-global-variables) the DMA writes the buffers, STM32CubeMonitor reads
// the rest
static std::array<uint8_t, bluetooth_rx_buffer_size> rx_buffer;
static std::array<uint8_t, bluetooth_tx_buffer_size> tx_buffer;

static volatile uint32_t test_baud_rate{};
static volatile bool     test_module_answered{};
static volatile uint32_t test_bytes_received{};
static volatile uint32_t test_bytes_sent{};
static volatile uint32_t test_lines_received{};
static volatile uint32_t test_bytes_refused{};
static volatile uint32_t test_link_events{};

// NOLINTEND(cppcoreguidelines-avoid-non-const-global-variables)

static std::span<const uint8_t> as_bytes(std::string_view text) {
    return {std::bit_cast<const uint8_t*>(text.data()), text.size()};
}

static uint32_t parse(std::string_view text) {
    uint32_t          value{};
    const char* const first = std::to_address(text.begin());
    const char* const last = std::to_address(text.end());
    const auto [end, error] = std::from_chars(first, last, value);
    return (error == std::errc{} and end == last) ? value : 0;
}

static bool send(proxy::BluetoothSerial& serial, std::string_view text) {
    if (serial.write(as_bytes(text)) == 0) {
        test_bytes_refused = test_bytes_refused + text.size();
        return false;
    }

    test_bytes_sent = test_bytes_sent + text.size();
    return true;
}

static std::size_t receive(proxy::BluetoothSerial& serial, std::span<uint8_t> into) {
    const std::size_t size = serial.read(into);
    test_bytes_received = test_bytes_received + size;
    return size;
}

static void wait_for_tx(proxy::BluetoothSerial& serial) {
    const proxy::Stopwatch stopwatch;
    serial.update();

    while (huart4.hdmatx->State == HAL_DMA_STATE_BUSY and stopwatch.elapsed_time_ms() < reply_timeout_ms) { }

    proxy::Stopwatch::sleep_ms(byte_time_ms);
}

static void set_baud_rate(proxy::BluetoothSerial& serial, uint32_t baud_rate) {
    wait_for_tx(serial);
    HAL_UART_DMAStop(&huart4);
    HAL_UART_AbortTransmit(&huart4);
    huart4.Init.BaudRate = baud_rate;
    HAL_UART_Init(&huart4);
    test_baud_rate = baud_rate;
}

static std::string ask(proxy::BluetoothSerial& serial, std::string_view text) {
    std::array<uint8_t, 64> chunk{};

    while (receive(serial, chunk) > 0) { }

    proxy::Stopwatch::sleep_ms(reply_quiet_ms);

    while (receive(serial, chunk) > 0) { }

    std::string reply;

    if (not send(serial, text)) {
        return reply;
    }

    const proxy::Stopwatch stopwatch;
    uint32_t               last_byte_ms{};

    while (stopwatch.elapsed_time_ms() < reply_timeout_ms) {
        serial.update();
        const std::size_t size = receive(serial, chunk);

        if (size > 0) {
            reply.append(std::bit_cast<const char*>(chunk.data()), size);
            last_byte_ms = stopwatch.elapsed_time_ms();
        } else if (not reply.empty() and stopwatch.elapsed_time_ms() - last_byte_ms > reply_quiet_ms) {
            break;
        }
    }

    return reply;
}

static std::string printable(std::string_view text) {
    std::string result;

    for (const char character : text) {
        if (character == '\r' or character == '\n') {
            continue;
        }

        result += (character >= ' ' and character <= '~') ? character : '.';
    }

    return result;
}

static std::string lowercase(std::string_view text) {
    std::string result{text};

    for (char& character : result) {
        if (character >= 'A' and character <= 'Z') {
            character = static_cast<char>(character - 'A' + 'a');
        }
    }

    return result;
}

static std::string probe(proxy::BluetoothSerial& serial) {
    test_module_answered = false;

    for (uint32_t round = 0; round < probe_rounds and not test_module_answered; round++) {
        for (const uint32_t baud_rate : baud_rates) {
            set_baud_rate(serial, baud_rate);

            if (ask(serial, "AT").contains("OK")) {
                test_module_answered = true;
                break;
            }
        }
    }

    if (not test_module_answered) {
        set_baud_rate(serial, baud_rates.front());
        return "module=silent\nuart=" + std::to_string(test_baud_rate) + "\n";
    }

    std::string report = "module=answered\nuart=" + std::to_string(test_baud_rate) + "\n";

    for (const std::string_view query : queries) {
        const std::string reply = ask(serial, "AT+" + std::string{query} + "?");
        report += lowercase(query) + "=" + (reply.empty() ? std::string{"(no reply)"} : printable(reply)) + "\n";
    }

    return report;
}

static std::string setting(proxy::BluetoothSerial& serial, std::string_view key, std::string_view command) {
    const std::string reply = ask(serial, command);
    return std::string{key} + "=" + (reply.empty() ? std::string{"(no reply)"} : printable(reply)) + "\n";
}

static std::string restart(proxy::BluetoothSerial& serial) {
    std::string result = setting(serial, "reset", "AT+RESET");
    proxy::Stopwatch::sleep_ms(module_restart_time_ms);
    return result;
}

static std::string configure(proxy::BluetoothSerial& serial, bool factory_reset) {
    std::string result;

    if (factory_reset) {
        result += setting(serial, "renew", "AT+RENEW");
        proxy::Stopwatch::sleep_ms(module_restart_time_ms);
        result += restart(serial);
        set_baud_rate(serial, baud_rates.front());
    }

    for (const Setting& entry : settings) {
        result += setting(serial, "set " + std::string{entry.key}, entry.command);
    }

#if __has_include("bluetooth_pin.hpp")
    result += setting(serial, "set pass", "AT+PASS" + std::string{bluetooth_pin});
    result += setting(serial, "set type", "AT+TYPE3");
#endif

    result += restart(serial);
    set_baud_rate(serial, configured_baud_rate);
    return result;
}

static std::string burst_line(uint32_t sequence, std::size_t size) {
    std::string line = "B " + std::to_string(sequence) + " ";

    for (std::size_t i = 0; i < size; i++) {
        line += static_cast<char>('a' + ((sequence + i) % 26));
    }

    line += "\n";
    return line;
}

namespace {
struct Burst {
    uint32_t         count{};
    uint32_t         next{};
    std::size_t      size{};
    proxy::Stopwatch stopwatch;
};
}  // namespace

static void start_burst(proxy::BluetoothSerial& serial, Burst& burst, std::string_view arguments) {
    const std::size_t separator = arguments.find(' ');

    if (separator == std::string_view::npos) {
        send(serial, "ERR usage BURST <count> <size>\n");
        return;
    }

    const uint32_t    count = parse(arguments.substr(0, separator));
    const std::size_t size = parse(arguments.substr(separator + 1));

    if (count == 0 or count > max_burst_count or size == 0 or size > max_burst_size) {
        send(
            serial, "ERR burst count 1.." + std::to_string(max_burst_count) + " size 1.." +
                        std::to_string(max_burst_size) + "\n"
        );
        return;
    }

    burst.count = count;
    burst.next = 0;
    burst.size = size;
    burst.stopwatch.reset_ms();
}

static void continue_burst(proxy::BluetoothSerial& serial, Burst& burst) {
    while (burst.next < burst.count) {
        const std::string line = burst_line(burst.next, burst.size);

        if (serial.write(as_bytes(line)) == 0) {
            return;
        }

        test_bytes_sent = test_bytes_sent + line.size();
        burst.next++;

        if (burst.next == burst.count) {
            send(
                serial,
                "BEND " + std::to_string(burst.count) + " " + std::to_string(burst.stopwatch.elapsed_time_ms()) + "\n"
            );
            burst.count = 0;
            return;
        }
    }
}

static void send_report(proxy::BluetoothSerial& serial, std::string_view report) {
    std::size_t start = 0;

    while (start < report.size()) {
        const std::size_t end = report.find('\n', start);

        if (end == std::string_view::npos) {
            break;
        }

        send(serial, "INFO " + std::string{report.substr(start, end - start)} + "\n");
        start = end + 1;
    }

    send(serial, "INFO END\n");
}

static bool handle_line(proxy::BluetoothSerial& serial, Burst& burst, std::string_view line, std::string_view report) {
    test_lines_received = test_lines_received + 1;

    if (line == "CONFIGURE") {
        send(serial, "OK configuring, the link drops now\n");
        return true;
    }

    if (line.starts_with("PING")) {
        send(serial, "PONG" + std::string{line.substr(4)} + "\n");
    } else if (line == "INFO") {
        send_report(serial, report);
    } else if (line.starts_with("BURST ")) {
        start_burst(serial, burst, line.substr(6));
    } else {
        send(serial, "ERR unknown " + printable(line) + "\n");
    }

    return false;
}

static void disconnect(proxy::BluetoothSerial& serial) {
    wait_for_tx(serial);
    proxy::Stopwatch::sleep_ms(reply_timeout_ms);
    ask(serial, "AT");
    proxy::Stopwatch::sleep_ms(module_restart_time_ms);
}

static void announce(proxy::Buzzer& buzzer, bool answered) {
    if (answered) {
        buzzer.play(1000, 100);
        buzzer.wait(50);
        buzzer.play(2000, 100);
        buzzer.wait(0);
    } else {
        buzzer.play(400, 600);
        buzzer.wait(0);
    }
}

int main(int argc, char* argv[]) {
    TestCore::init(argc, argv);
    proxy::BluetoothSerial serial{bluetooth_config, rx_buffer, tx_buffer};
    proxy::Button          button{button_config};
    proxy::Buzzer          buzzer{buzzer_config};
    proxy::Led             led{led_config};

    led.turn_on();
    proxy::Stopwatch::sleep_ms(module_boot_time_ms);
    std::string report = probe(serial);
    announce(buzzer, test_module_answered);

    std::string             line;
    bool                    configure_requested{};
    Burst                   burst;
    proxy::Stopwatch        heartbeat;
    proxy::Stopwatch        blink;
    std::array<uint8_t, 64> chunk{};

    TestCore::loop([&]() {
        serial.update();
        button.update();

        const std::size_t size = receive(serial, chunk);

        for (std::size_t i = 0; i < size; i++) {
            const char character = static_cast<char>(chunk.at(i));

            if (character == '\n') {
                configure_requested = handle_line(serial, burst, line, report) or configure_requested;
                line.clear();
            } else if (character != '\r') {
                line += character;

                for (const std::string_view event : module_events) {
                    if (line.ends_with(event)) {
                        line.resize(line.size() - event.size());
                        test_link_events = test_link_events + 1;
                    }
                }

                if (line.size() > max_line_size) {
                    send(serial, "ERR line too long\n");
                    line.clear();
                }
            }
        }

        const proxy::Button::Status status = button.get_status();

        if (status != proxy::Button::Status::NO_PRESS or configure_requested) {
            led.turn_on();

            if (configure_requested) {
                disconnect(serial);
            }

            const bool        changing = configure_requested or status == proxy::Button::Status::LONG_PRESS or
                                         status == proxy::Button::Status::EXTRA_LONG_PRESS;
            const std::string changes = (changing and test_module_answered) ?
                                            configure(serial, status == proxy::Button::Status::EXTRA_LONG_PRESS) :
                                            "";
            report = changes + probe(serial);
            announce(buzzer, test_module_answered);
            line.clear();
            configure_requested = false;
        }

        continue_burst(serial, burst);

        if (heartbeat.elapsed_time_ms() >= heartbeat_interval_ms) {
            heartbeat.reset_ms();
            send(
                serial, "HB " + std::to_string(HAL_GetTick()) + " rx=" + std::to_string(test_bytes_received) +
                            " lines=" + std::to_string(test_lines_received) + " refused=" +
                            std::to_string(test_bytes_refused) + " links=" + std::to_string(test_link_events) + "\n"
            );
        }

        if (blink.elapsed_time_ms() >= (test_module_answered ? 500U : 100U)) {
            blink.reset_ms();
            led.toggle();
        }
    });

    return 0;
}
