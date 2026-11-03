# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What This Is

Paula is an ESP32-based LoRa receiver and display hub for the Digital Stables farm monitoring system. It sits at the center of a LoRa mesh, receives sensor telemetry from multiple remote nodes, and presents data on an SSD1306/SSD1309 OLED display with WS2812 LED status indicators.

## Build & Flash

This is an Arduino sketch. Use Arduino IDE or `arduino-cli`:

```bash
# Compile
arduino-cli compile --fqbn esp32:esp32:esp32 Paula.ino

# Flash (adjust port as needed)
arduino-cli upload --fqbn esp32:esp32:esp32 --port /dev/ttyUSB0 Paula.ino

# Monitor serial output
arduino-cli monitor --port /dev/ttyUSB0 --config baudrate=115200
```

Required board package: `esp32:esp32` (Espressif Arduino core).

## Hardware Pinout

| Function | Pin |
|---|---|
| LoRa SCK/MOSI/MISO/SS | 14/13/12/15 |
| LoRa RESET/DI0 | 16/17 |
| WS2812 LEDs (6x, 2 rows of 3) | 19 |
| RTC CLK_OUT (1Hz interrupt) | 4 |
| Left switch | 32 |
| Right switch | 33 |
| Sleep switch | 26 |
| OLED (I2C) | default I2C pins, addr 0x3C |

LoRa: 433 MHz, SF=9, BW=125 kHz, CRC enabled.

## Architecture

### Packet Identification
Incoming LoRa packets are identified **solely by byte size** — each data struct (`LangleyData`, `DigitalStablesData`, `ChinampaData`, `GloriaTankFlowPumpData`, `SeedlingMonitorData`, `WeatherForecastUpdate`) must have a unique `sizeof()`. This is a hard constraint: if you add fields to any struct, verify no two structs collide in size.

### Data Flow
1. LoRa ISR (`onReceive`) sets `loraReceived` flag + packet size (never reads bytes in ISR)
2. `loop()` calls `processLora()` which reads bytes and dispatches to the correct struct
3. Received structs are stored via `DataManager` to LittleFS, and display/LED state is updated immediately
4. A `DataManager` queue is drained when the host sends `AsyncData` over serial

### Display Logic
- Switch LEFT (`switchPositionLeft == true`): shows incoming LoRa data — Chinampa page 1, TopTank, weather when received
- Switch RIGHT (`switchPositionLeft == false`): locks display to `showWeatherForecast()` regardless of incoming packets
- `displayingChinampa` is only cleared on non-Chinampa packets when switch is LEFT
- `refreshChinampaPage` flag is set on switch change; only redraws chinampa page 1 when switch is left

### LED Meanings
| LED index | Meaning |
|---|---|
| 0 | WiFi: green=connected, dim-red=no IP |
| 1 | LoRa: green=module active, red=init failed |
| 2 | Weather freshness: green=<2h, red=stale |
| 3 | Weather freshness (duplicate of 2): green=<2h, red=stale |
| 4,5 | Activity: green on any LoRa receive; light-blue on WeatherForecastUpdate; cleared each 1 Hz tick |

### RTC Clock Tick
The PCF8563 RTC outputs a 1 Hz square wave to pin 4, handled by `clockTick()` ISR using a `portMUX_TYPE` critical section. All time-sensitive per-second work (staleness tracking, switch polling, time refresh) runs inside `if (clockTicked)` in `loop()`.

### Serial Command Interface
Paula accepts commands over serial (115200 baud) for host integration:
- `SetTime#d#m#yy#dow#hh#mm#ss` — set RTC
- `GetTime` — print current time
- `GetCommandCode` — generate TOTP code (zero-pads if <100000)
- `SetSecret#base32#digits#period` — configure TOTP secret
- `GetSecret` — read stored TOTP secret
- `AsyncData` — drain DataManager queues (returns stored DigitalStables/Chinampa/Gloria data)
- `GetSensorData` — serialize last `DigitalStablesData` to serial
- `IPAddr#x.x.x.x` — update stored IP address
- `PulseStart` / `PulseFinished` — bracket a pulse event
- `debug#1` / `debug#0` — toggle serial debug output

### WeatherForecastUpdate Authentication
`WeatherForecastUpdate` packets are TOTP-authenticated via `Esp32SecretManager`. The sender must include a valid TOTP code in `wfu.totpcode`; invalid codes are silently dropped.

## Key Custom Libraries (not in this repo)

| Library | Purpose |
|---|---|
| `DigitalStablesData` | Septic tank height + battery monitor struct |
| `ChinampaData` | Aquaponics (fish tank, sump, pump, solenoid) struct |
| `LangleyData` | Weather station struct |
| `GloriaTankFlowPumpData` | Water tank flow/pump struct |
| `SeedlingMonitoringData` | Seedling humidity/temp struct |
| `SolarPowerData` | Solar power struct (included, unused in display logic) |
| `PCF8563TimeManager` | I2C RTC driver + NTP-style time management |
| `Esp32SecretManager` | TOTP secret storage + code generation/verification |
| `DataManager` | LittleFS queue storage + serial serialization |
| `GeneralFunctions` | `getValue(str, delimiter, index)` token parser |
| `TimeUtils` | `epochToString()`, `parseTimezone()` |
