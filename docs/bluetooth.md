# Bluetooth link

The robot exposes its variables over an HM-19 BLE module wired to `UART4` (`PA11` = RX,
`PA12` = TX) through the USB-C receptacle. The protocol that runs over it is micras-lib's
[micras_comm](../external/micras-lib/micras_comm/README.md), and the companion application is
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
