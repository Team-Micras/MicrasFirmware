# Bluetooth link

The robot exposes its variables over an HM-19 BLE module wired to `UART4` (`PA11` = RX,
`PA12` = TX) through the USB-C receptacle. The protocol that runs over it is micras-lib's
[micras_comm](https://github.com/Team-Micras/micras-lib/blob/refactor/restructure/micras_comm/README.md),
and the companion application is [micras-monitor](https://github.com/Team-Micras/micras-monitor).

## Wiring

The module's serial port *is* the USB-C receptacle. `D-` reaches `PA11` and `D+` reaches `PA12`,
each through a 33 Ω series resistor, and `SBU1`/`SBU2` supply 3V3 through a 300 mA resettable fuse.
`CC1` and `CC2` terminate in 5.1 kΩ resistors and never reach the microcontroller.

One consequence of that shapes the protocol: **there is no hardware flow control.** The CC2640R2F
inside the module has `RTS`/`CTS`, but the board cannot reach them, and every other conductor of the
receptacle is already used. The module therefore drops bytes silently when its buffer fills, which
is what the credit window in the session layer exists to prevent.

`PA11` and `PA12` are also `USB_OTG_HS_DM` and `USB_OTG_HS_DP`, and the board carries the 5.1 kΩ
`CC` terminations and the 1.5 kΩ `D+` pull-up of a full speed USB device, so the same receptacle
can be a USB port instead. It cannot be both at once, and the radio owns it.

## Configuring the module

The module needs these once, over AT commands, with no application connected. Send them without a
line ending and wait for each reply. `test_bluetooth` sends them on a long press of the button or on
a `CONFIGURE` line from a connected application.

| Command | Reply | Why |
| --- | --- | --- |
| `AT` | `OK` | Check that the module is in command mode. |
| `AT+ROLE0` | `OK+Set:0` | Peripheral. The application is the central. |
| `AT+NAMEMicras` | `OK+Set:Micras` | What the application looks for. |
| `AT+MODE0` | `OK+Set:0` | Forward everything the application writes. The factory mode executes writes that start with `AT`, which a binary frame can. |
| `AT+NOTI0` | `OK+Set:0` | Keep `OK+CONN` and `OK+LOST` out of the serial stream. |
| `AT+COMI0` | `OK+Set:0` | Ask for a 7.5 ms minimum connection interval. |
| `AT+COMA0` | `OK+Set:0` | Ask for a 7.5 ms maximum connection interval. |
| `AT+BAUD7` | `OK+Set:7` | 115 200, matching `MX_UART4_Init`. The index is the HM-19 table, where `4` is 19 200. |
| `AT+PASS<pin>` | `OK+Set:<pin>` | The six digit PIN, from `config/bluetooth_pin.hpp`. |
| `AT+TYPE3` | `OK+Set:3` | Ask for the PIN and bond, so each computer enters it once. |
| `AT+RESET` | `OK+RESET` | Apply. |

`config/bluetooth_pin.hpp` is not under version control, so the PIN stays out of the repository.
Without it, `test_bluetooth` leaves the PIN and the bond mode alone. The file declares a single
`micras::bluetooth_pin`:

```cpp
constexpr std::string_view bluetooth_pin{"123456"};
```

The module keeps its side of every bond. A computer that forgets the pairing cannot pair again until
the module is restored with an extra long press of the button, which also applies the configuration
again.

`AT+COMI` and `AT+COMA` are a request: the central decides the interval it actually uses, and a
browser gives the page no way to read or influence it. The throughput is therefore only knowable by
measuring it, which the `link/dropped_samples` counter and the sample sequence numbers make easy.

Raising the serial speed to 230 400 is `AT+BAUD8` here and `Dma.UART4_RX` plus the baud rate in
`cube/micras_v1.ioc` on the firmware side. It is only worth doing once a measurement shows the
serial port, rather than the radio, is the limit.

The application connects to service `FFE0`, characteristic `FFE1`, with notifications enabled.

## Writing to the module

The module forwards a write to its serial port some time after it arrives, from the characteristic's
single value. A write that arrives before the previous one was forwarded replaces it, and the module
then sends the latest value once for each write: two 20 byte writes in a row come out as the second
one twice. Each message therefore goes in **one** write without response, and writes are spaced:
back to back, 3 of 20 messages arrived corrupted at 115 200, while 20 ms or more between writes
delivered every one. The connection negotiates a 247 byte MTU, so a write carries up to 244 bytes. The
module refuses writes with response.

Windows ignores the interval the module asks for and uses 60 ms. An application that can, as a native
Windows one through `RequestPreferredConnectionParameters(ThroughputOptimized)`, gets 15 ms.

The factory settings of the fitted firmware (`HMSoft V117`) include `AT+NOTI1`, so until it is
configured the module writes `OK+CONN` and `OK+LOST` to the serial port, without a line ending,
whenever a link opens or closes.

In the other direction the serial port at 115 200 outruns the radio. Unpaced bursts from the robot
reached the PC at 6–11 kB/s with 14–46 % of the lines lost or cut at the 60 ms interval, which is the
loss the credit window prevents. At 9 600 the serial port is the limit and nothing is lost.

## Checking the link without the protocol

`test_bluetooth` exercises the radio through `proxy::BluetoothSerial` with plain text lines. It
changes the module only when asked to: a long press (or a `CONFIGURE` line) applies the configuration
above, and an extra long press restores the factory settings with `AT+RENEW` first.

1. With no application connected, it looks for the module's baud rate by sending `AT` at 9 600
   (the factory setting), 115 200 and the other rates the module supports, then reads its settings
   with read-only queries (`AT+VERS?`, `AT+ADDR?`, `AT+NAME?`, `AT+BAUD?`, …). Two rising beeps and
   a slow blink mean the module answered, one low beep and a fast blink mean it stayed silent. A
   silent module leaves the port at 9 600, so the radio side can still be checked.
2. It then answers `PING <text>` with `PONG <text>`, `INFO` with the report of step 1,
   `BURST <lines> <size>` with numbered lines of a known pattern followed by `BEND`, and sends a
   `HB` heartbeat every second. A short press of the button repeats step 1, which disconnects a
   connected application, since `AT` is also the module's disconnect command.

`tools/bluetooth_check.html` is the other end. Web Bluetooth needs a secure origin, so serve it from
WSL and open it in Chrome or Edge on Windows:

```sh
python3 -m http.server 8000 -d tools
# http://localhost:8000/bluetooth_check.html
```

An unconfigured module advertises as `HMSoft`. Each check isolates one segment of the path:

| Check | Fails when |
| --- | --- |
| Service FFE0 / characteristic FFE1 | The PC cannot reach the module at all |
| Heartbeat | Robot → module is broken: `PA12`, the module's `UART_RX`, or a baud mismatch |
| `AT` at boot | Module → robot is broken: the module's `UART_TX` or `PA11`, if the heartbeat passes |
| Ping echo | Either direction drops or corrupts bytes under request–reply traffic |
| Burst | Bulk robot → PC traffic is lost. It passes at 9 600 and measures the loss at 115 200 |

Lines from the PC never start with `AT`: an unconfigured module executes them as commands over the
air.
