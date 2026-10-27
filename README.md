<p align="center"><img src="assets/sdlview.png" width="96" alt="SDLView logo"></p>

# Suzuki-SDL-Viewer

**SDLView** is a lightweight, portable Windows tool that reads live data and fault codes from older Suzuki engine ECUs over the **SDL** (Serial Data Line) diagnostic protocol, using a cheap FTDI-based KKL USB cable.

It is built for and tested on a **1997 Suzuki Swift 1.0 (G10A, TBI, automatic) with SDL ECU ID `1453`**. Other SDL ECUs will connect, but addresses, formulas and fault codes may differ; the app warns when the ECU ID is not `1453`.


## Features

| | |
|---|---|
| **Dashboard** | Engine speed, target idle, coolant and intake air temperature, MAP and barometric pressure, throttle, ISC position, injector pulse, ignition advance, battery voltage, closed-throttle switch, radiator fan, O2 sensor (amber = rich, blue = lean), short- and long-term fuel trim, A/T throttle level, and an **RPM glitch** counter (spikes in the engine speed signal, amber at 3 or more per minute; a wrong distributor rotor caused this on the test car). Values whose formula is not yet confirmed are marked ⚠ |
| **Graph** | Four stacked live line charts. Each channel shows any converted value (including the unfiltered engine speed) or any raw address `0x00`–`0x2F`. Time window 10 s – 5 min, auto scale, pause |
| **Fault codes** | Current and stored codes with descriptions |
| **Raw** | All 48 addresses as decimal, hex and bits; changes are highlighted |
| **Logging** | CSV with every raw byte plus the converted values |
| **Idle command** | Temporarily sets the ECU's target idle speed (600–1600 rpm). It is restored automatically on disconnect, on closing, above 2500 rpm, or when the engine stalls. Unconfirmed restores are retried and reported |

## Hardware

- **Cable:** "VAG KKL 409.1" USB cable with a genuine **FTDI FT232R** chip (needed for the non-standard 7812 baud).
- **Connector:** on this car the SDL line is on **pin 9** of the 16-pin diagnostic connector, while the KKL cable reads its K-line on **pin 7**. Bridge pin 9 to pin 7 (for example with an OBD2 breakout box).

## Usage

1. Download `SDLView.exe` from the [Releases](../../releases) page (no installation needed).
2. Plug in the cable and switch the ignition on. Close any other program that uses the cable.
3. Start `SDLView.exe`.
4. Click **Connect**.

## Protocol summary (SDL, ECU 1453)

**Serial:** 7812 baud, 8N1. The cable is half-duplex, so every transmitted byte is echoed back before the ECU replies.

**Frame:** `[header] [length] [data …] [checksum]`, with `length = 3 + len(data)` and `checksum = (-sum(header … data)) & 0xFF`. The reply uses the same header.

| Header | Request data | Reply data | Use |
|---|---|---|---|
| `0x10` | – | 2 bytes | ECU ID (`14 53` → `1453`) |
| `0x13` | list of addresses (max 37) | 1 byte per address | Read live data (`0x00`–`0x2F` respond) |
| `0x15` | 8 bytes | – (ack `15 03 E8`) | Actuator. Idle target: `C0 <rpm/7.84375> 00 00 00 00 00 00`. Restore: eight `00` bytes |

| Address | Value | Formula | Status |
|---|---|---|---|
| `0x00` / `0x01` | Current fault bits | see fault table | confirmed |
| `0x02` / `0x03` | Stored fault bits | same layout as `0x00` / `0x01` | confirmed |
| `0x04` / `0x05` | Engine speed | `((hi & 0x3F)·256 + lo) · 0.195`; skip samples with `hi & 0xC0` set | confirmed |
| `0x06` | Target idle | `x · 7.84375` rpm | confirmed |
| `0x07` | Vehicle speed | `x` km/h | |
| `0x08` / `0x09` | Coolant / intake air temp | `x · 0.625 − 40` °C | confirmed |
| `0x0A` / `0x0B` | Throttle angle / voltage | raw / `x · 5/256` V | confirmed |
| `0x0C` | O2 sensor (bank 1) | `x · 5/256` V; switches ~0.1–0.9 V at ~1 Hz when warm and in closed loop | confirmed |
| `0x0D` / `0x0E` | Injector pulse | `(hi·256 + lo) · 0.002` ms; about 1 in 4 samples reads `0`, so use the maximum over ~1 s | confirmed |
| `0x0F` | Ignition advance | `x · 0.35 − 11.7` ° BTDC | confirmed |
| `0x10` / `0x11` | MAP / barometric | `x · 6.4844 − 193.33` mbar | confirmed |
| `0x12` | ISC position: actual throttle opening within the ISC actuator range | `x/255 · 100` %; follows the TPS, so it is a measured position, not a command | confirmed |
| `0x13` | Short-term fuel trim | `(x − 128)/128 · 100` %; rises when the O2 sensor reads lean, falls when it reads rich | address confirmed, scale unconfirmed |
| `0x14` | Long-term fuel trim | `(x − 128)/128 · 100` %; stayed at `128` in every capture so far | unconfirmed |
| `0x15` | Fuel trim (total?) | raw, 128 = neutral; follows `0x13` (equal in about half of the samples) | unconfirmed |
| `0x16` | Battery | `x · 20/256` V | confirmed |
| `0x18` | Status | bit 4 = closed-throttle switch | confirmed |
| `0x19` | Radiator fan | `128` = on; stayed `0` up to 84 °C coolant (fan switches on at 97.5 °C) | |
| `0x23` | Throttle opening level sent to the A/T | `0`–`7`; `0` at closed throttle, rises with throttle opening (`0`–`6` seen) | |

"Confirmed" means checked on the car against measurements, a multimeter or a reference scan tool.

| Byte / bit | Code | Fault |
|---|---|---|
| 0 / 0 | 13 | Oxygen (lambda) sensor |
| 0 / 1, 0 / 2 | 14, 15 | Coolant temperature sensor high / low |
| 0 / 3, 0 / 4 | 21, 22 | Throttle position sensor high / low |
| 0 / 5, 0 / 7 | 23, 25 | Intake air temperature sensor high / low |
| 0 / 6 | 24 | Vehicle speed sensor |
| 1 / 0, 1 / 1 | 31, 32 | MAP sensor high / low |
| 1 / 2 | 41 | Ignition fail-safe signal |
| 1 / 3 | 42 | Crank angle sensor (in the distributor) |
| 1 / 4 | 51 | EGR system |

## Credits

- [thatdevsherry/suzuki_sdl](https://github.com/thatdevsherry/suzuki_sdl): the initial SDL address map and frame format (Suzuki Baleno).
- Suzuki Swift SF310/SF413 service manual: idle control, CTP switch and ignition specifications.

## License

[MIT](LICENSE)
