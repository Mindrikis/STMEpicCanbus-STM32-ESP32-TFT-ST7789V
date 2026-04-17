# ESP32 ST7789 LVGL dashboard

**PlatformIO** + **Arduino-ESP32**: 240×320 **ST7789** (SPI), **LVGL 9**, **TFT_eSPI**. Receives telemetry and button bytes from the **STM32 gateway** on **UART2**.

## Requirements

- Board env: **`nodemcu-32s`** (default) — see `platformio.ini`
- Libraries: **TFT_eSPI**, **LVGL** (`lib_deps`)
- TFT: user **TFT_eSPI** `User_Setup` must match pins / `ST7789` / **240×320** (build flags set driver + size here)

## Build / flash / monitor

```bash
pio run -e nodemcu-32s -t upload
pio device monitor -b 115200
```

If upload fails: disconnect gateway from **GPIO25/26**, stable USB, try `nodemcu-32s-slow` env.

## How it works

| Piece | Role |
|-------|------|
| `Serial2` | Gateway link **115200** RX=`GPIO25` TX=`GPIO26`; `setRxBufferSize` **before** `begin` |
| `process_mega_commands` | Parses `F D T …` lines + queues `N P O Q S R` for LVGL-safe handling |
| `main.cpp` UI | Manual tab stack (no `lv_tabview`); menus as overlays; trip reset: **`S`** or **`R`** on dashboard |
| Splash | Full-screen LVGL splash then deferred `build_main_ui()` |
| Loop stack | `ARDUINO_LOOP_STACK_SIZE` **16 KiB** (before `Arduino.h`) for large LVGL build |

## Protocol (gateway → ESP32)

Same newline ASCII as STM32 `README.md`: fuel/trip/boost/inj/AFR/MAP/IAT/CLT/speed/RPM, accel **`a–d`**, GPS **`Y Z H g`**, status **`R U K`**.

## Config

- **Pins / SPI / fonts:** `platformio.ini` `build_flags`
- **UART, sensors, buzzer, backlight PWM:** `src/main.cpp` `#define`s (or `-D` overrides)

## Related repo

**STM32 CAN gateway:** `README.md` + **`README_PINMAP.md`** (USART3 ↔ this board).

**Wiring table:** **`README_PINMAP.md`** (this project).
