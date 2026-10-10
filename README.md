<p align="center"><img src="assets/sdlview.png" width="96" alt="SDLView logo"></p>

# Suzuki-SDL-Viewer

**SDLView** is a lightweight, portable Windows tool that reads live data and fault codes from older Suzuki engine ECUs over the **SDL** (Serial Data Line) diagnostic protocol, using a cheap FTDI-based KKL USB cable.

It is built for and tested on a **1997 Suzuki Swift 1.0 (G10A, TBI, automatic) with SDL ECU ID `1453`**. Other SDL ECUs will connect, but addresses, formulas and fault codes may differ; the app warns when the ECU ID is not `1453`.

> **Disclaimer.** This is a hobby project, not affiliated with Suzuki. The *Idle command* changes how the ECU controls the engine. Use it only with the vehicle parked, and entirely at your own risk.

## Features

| | |
|---|---|
| **Dashboard** | Engine speed, target idle, coolant and intake air temperature, MAP and barometric pressure, throttle, ISC duty, injector pulse, ignition advance, battery voltage, closed-throttle switch, radiator fan, mixture control. Values whose formula is not yet confirmed are marked ⚠ |
| **Graph** | Four stacked live line charts. Each channel shows any converted value or any raw address `0x00`–`0x2F`. Time window 10 s – 5 min, auto scale, pause |
| **Fault codes** | Current and stored codes with descriptions |
| **Raw** | All 48 addresses as decimal, hex and bits; changes are highlighted |
| **Logging** | CSV with every raw byte plus the converted values |
| **Idle command** | Temporarily sets the ECU's target idle speed (600–1600 rpm). It is restored automatically on disconnect, on closing, above 2500 rpm, or when the engine stalls. Unconfirmed restores are retried and reported |

## Hardware

- **Cable:** "VAG KKL 409.1" USB cable with a genuine **FTDI FT232R** chip (needed for the non-standard 7812 baud).
- **Connector:** on this car the SDL line is on **pin 9** of the 16-pin diagnostic connector, while the KKL cable reads its K-line on **pin 7**. Bridge pin 9 to pin 7 (for example with a OBD2 break out box).

## Usage

1. Plug in the cable and switch the ignition on. Close any other program that uses the cable.
2. Start `SDLView.exe`.
3. Click **Connect**.

Logs are written to `logs\` next to the executable, or to `Documents\SDLView logs` if that folder is not writable. Windows SmartScreen may warn about an unsigned app the first time: *More info → Run anyway*.

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
| `0x04` / `0x05` | Engine speed | `((hi & 0x3F)·256 + lo) / 5.1`; skip samples with `hi & 0xC0` set | confirmed |
| `0x06` | Target idle | `x · 7.84375` rpm | confirmed |
| `0x07` | Vehicle speed | `x` km/h | |
| `0x08` / `0x09` | Coolant / intake air temp | `x/255 · 159 − 40` °C | confirmed |
| `0x0A` / `0x0B` | Throttle angle / voltage | raw / `x/255 · 5` V | confirmed |
| `0x0D` / `0x0E` | Injector pulse | `(hi·256 + lo) · 0.002` ms | confirmed |
| `0x0F` | Ignition advance | `x/255 · 90 − 12` ° (correct with engine off only) | unconfirmed |
| `0x10` / `0x11` | MAP / barometric | `x/255 · 1666.3 − 200` mbar (≈3 mbar high) | unconfirmed |
| `0x12` | ISC duty | `x/255 · 100` % | confirmed |
| `0x13`–`0x15` | Mixture control | raw, 128 = neutral | unconfirmed |
| `0x16` | Battery | `x · 20/256` V | confirmed |
| `0x18` | Status | bit 4 = closed-throttle switch | confirmed |
| `0x19` | Radiator fan | `128` = on | |

| Byte / bit | Code | Fault |
|---|---|---|
| 0 / 0 | 13 | Oxygen (lambda) sensor |
| 0 / 1, 0 / 2 | 14, 15 | Coolant temperature sensor high / low |
| 0 / 3, 0 / 4 | 21, 22 | Throttle position sensor high / low |
| 0 / 5, 0 / 7 | 23, 25 | Intake air temperature sensor high / low |
| 0 / 6 | 24 | Vehicle speed sensor |
| 1 / 0, 1 / 1 | 31, 32 | MAP sensor high / low |
| 1 / 2 | 41 | Ignition fail-safe signal |
| 1 / 3 | 42 | Crank angle sensor |
| 1 / 4 | 51 | EGR system |

"Confirmed" means checked on the car against measurements, a multimeter or a reference scan tool.

## Credits

- [thatdevsherry/suzuki_sdl](https://github.com/thatdevsherry/suzuki_sdl): the initial SDL address map and frame format (Suzuki Baleno).
- Suzuki Swift SF310/SF413 service manual: idle control, CTP switch and ignition specifications.

## License

[MIT](LICENSE)
