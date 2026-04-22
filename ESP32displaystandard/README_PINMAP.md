# ESP32 display — pins & wiring

Defaults: **`platformio.ini`** (TFT SPI) + **`src/main.cpp`** (UART, GPIO). Change flags **and** code (or `-D` overrides) together.

## Board

- **Env:** `nodemcu-32s` (ESP32-WROOM-32 class)

## ST7789 — SPI (`build_flags`)

| Signal | GPIO |
|--------|------|
| MISO | 19 |
| MOSI | 23 |
| SCLK | 18 |
| CS | 15 |
| DC | 2 |
| RST | 4 |

- Driver **ST7789_2**, **240×320**, **`SPI_FREQUENCY=27000000`** (see `platformio.ini`), **`TFT_BGR`**
- **Backlight PWM:** default **`GPIO33`** (`TFT_BACKLIGHT_PIN`) — wire panel `BL` to this pin (not bare 3V3 if you want dimming)

Avoid strapping **GPIO0** / **2** / **15** in ways that block flash boot (see `platformio.ini`).

## UART2 — gateway (STM32)

| ESP32 | To STM32 |
|-------|-----------|
| **GPIO25** RX | `PB10` (USART3 **TX**) |
| **GPIO26** TX | `PB11` (USART3 **RX**) |
| **GND** | GND |

**115200 8N1** · TTL cross-over as above · **common GND**

Debug wiring: `-D DASH_LINK_SNIFFER=1` → mirror Serial2 RX to USB `Serial`.

## Local GPIO (`src/main.cpp`)

| Function | Default GPIO | Macro | Sense |
|----------|----------------|--------|--------|
| Oil pressure OK | 27 | `OIL_PRESSURE_SWITCH_PIN` | Active-**low** = OK (pull to GND when pressure OK) |
| Coolant level OK | 13 | `COOLANT_LEVEL_PIN` | **HIGH** = OK (internal pull-up; **GND** = low fluid → warning) |
| Brake fluid OK | 14 | `BRAKE_FLUID_PIN` | Same as coolant |

Oil + coolant + brake **warnings** (strip, bottom bar, buzzer) only latch after the pin reads **fault** continuously for **~2 s** (`DASH_SENSOR_FAULT_CONFIRM_MS`, override e.g. `-D DASH_SENSOR_FAULT_CONFIRM_MS=1500`).

## Buzzer

| Macro | Default |
|--------|---------|
| `BUZZER_PIN` | **16** (avoid **GPIO5** — VSPI SS / SPI clash) |
| `BUZZER_ACTIVE_HIGH` | 1 |

## UART0

USB **Serial** — upload / monitor only. For reliable flash, idle gateway on **25/26**.

## STM32 reference

**`STM32 canbus/README_PINMAP.md`** — CAN, GPS `PD5/PD6`, cruise, EPIC IDs.
