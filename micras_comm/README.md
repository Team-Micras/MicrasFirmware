# Bluetooth link

The robot exposes its variables over an HM-19 BLE module wired to `UART4` (`PA11` = RX,
`PA12` = TX) through the USB-C receptacle. The companion application is
[micras-monitor](https://github.com/Team-Micras/micras-monitor).

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
line ending and wait for each reply.

| Command | Reply | Why |
| --- | --- | --- |
| `AT` | `OK` | Check that the module is in command mode. |
| `AT+ROLE0` | `OK+Set:0` | Peripheral. The application is the central. |
| `AT+NAMEMicras` | `OK+Set:Micras` | What the application looks for. |
| `AT+BAUD4` | `OK+Set:4` | 115 200, matching `MX_UART4_Init`. |
| `AT+COMI0` | `OK+Set:0` | Ask for a 7.5 ms minimum connection interval. |
| `AT+COMA0` | `OK+Set:0` | Ask for a 7.5 ms maximum connection interval. |
| `AT+RESET` | `OK+RESET` | Apply. |

`AT+COMI` and `AT+COMA` are a request: the central decides the interval it actually uses, and a
browser gives the page no way to read or influence it. The throughput is therefore only knowable by
measuring it, which the `link/dropped_samples` counter and the sample sequence numbers make easy.

Raising the serial speed to 230 400 is `AT+BAUD8` here and `Dma.UART4_RX` plus the baud rate in
`cube/micras_v1.ioc` on the firmware side. It is only worth doing once a measurement shows the
serial port, rather than the radio, is the limit.

The application connects to service `FFE0`, characteristic `FFE1`, with notifications enabled.

## Protocol

Frames are [COBS](https://en.wikipedia.org/wiki/Consistent_Overhead_Byte_Stuffing) encoded and
delimited by a zero byte:

```text
COBS( type | payload | fletcher16 )  0x00
```

The frame check is Fletcher-16 over the type and the payload, before encoding, little endian. It is
not protecting against the radio, which has a CRC-24 and retransmits until acknowledged. It covers
the two hops BLE never sees: the 8N1 serial port with no parity between the microcontroller and the
module, and the module's buffer, which drops runs of bytes with no indication that it did. A
dropped run can join the head of one frame to the tail of the next into something with a plausible
shape, and the two running sums make that about as unlikely as a sixteen bit check can: one over
sixty five thousand, against one in two hundred and fifty five for a single sum of bytes. It is
weaker than a CRC on long bursts, which is what a CRC is uniquely good at, and it is four lines
that live beside the codec they belong to.

Message types, their payloads and the meaning of every field are in
`micras_comm/include/micras/comm/protocol.hpp`. Everything on the wire is little endian, which is
what both the microcontroller and `DataView` in the browser already are.

### The parts worth knowing before reading the code

- **The schema is fetched once per firmware build, not once per connection.** `HELLO_ACK` carries a
  `schema_hash` over every name, type and access flag in order. Identifiers are registration order,
  so adding one variable shifts every later one; comparing the hash against the one a cached schema
  was fetched with is what stops the application from plotting the wrong signal.
- **Samples come in groups, not one variable at a time.** A `SAMPLE` carries several variables
  captured in the same control loop iteration under one timestamp. A response plotted against a
  setpoint captured two iterations later is not a plot of a control loop.
- **The application has to return credit.** The robot may have at most `initial_credit` bytes
  outstanding. `CREDIT` says how many more bytes the application has taken. When the window is
  closed, samples are dropped and the sequence number shows the gap; replies to requests are not
  charged to the window, because they are already bounded by the rate of the requests themselves.
- **Writes are levels and commands are edges.** `WRITE` sets a gain or a flag and is acknowledged
  with a result; the `idle` flag on a variable refuses the dangerous ones while the robot is moving.
  `COMMAND` happens once, when it arrives.
- **The link cannot carry the control loop.** It is between twenty and a hundred times too slow for
  8 kHz, so a group is defined with a period in loop iterations and only every period-th iteration
  is sent. The rate the application asks for is a rate it can actually receive.
