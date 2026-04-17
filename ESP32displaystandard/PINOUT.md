# ESP32 <-> ST7789V Pinout

This project uses SPI with `TFT_eSPI` on an ESP32 (`nodemcu-32s`).

## Display Label to ESP32 Mapping

| Display pin | Meaning | ESP32 pin | Notes |
|---|---|---|---|
| `GND` | Ground | `GND` | Common ground required |
| `VCC` | Display power | `3V3` | Use 3.3V (recommended for ESP32 logic) |
| `SCL` | SPI clock | `GPIO18` | Same as `TFT_SCLK` |
| `SDA` | SPI MOSI (data to TFT) | `GPIO23` | Same as `TFT_MOSI` |
| `RST` | Display reset | `GPIO4` | Same as `TFT_RST` |
| `DC` | Data/Command select | `GPIO2` | Same as `TFT_DC` |
| `CS` | Chip select | `GPIO15` | Same as `TFT_CS` |
| `BL` | Backlight | **`GPIO33`** (PWM) | Firmware: **Menu → Brightness** + gateway **N**/**P**. Was 3V3 always-on; move wire to **GPIO33** (active high PWM). Override: `-D TFT_BACKLIGHT_PIN=…` |
| `SDA-0` | SPI MISO / SDO (optional) | `GPIO19` | Optional for readback; not required for normal drawing |

## UART Link to STM32 gateway (ESP32 Serial2)

Do **not** wire the gateway to **TX0 / RX0** if you want reliable **USB upload** and Serial Monitor: those pins are UART0 and conflict with the USB–UART bridge.

Firmware uses **`Serial2`** (UART2) on **GPIO25 / GPIO26** (`MEGA_SER_RX_PIN` / `MEGA_SER_TX_PIN` in `main.cpp` — legacy names).

| STM32F407 (Black) | MCU pin | ESP32 GPIO | `Serial2` role |
|---|---|---|---|
| **TX** | `PB10` | **GPIO25** | RX (ESP32 listens) |
| **RX** | `PB11` | **GPIO26** | TX (ESP32 sends) |
| GND | GND | GND | Required |

**Levels:** STM32 and ESP32 are both **3.3 V** logic — direct wiring is fine (no 5 V Mega shifter).

**Legacy Mega (Serial3):** Mega **TX3** (D14) → GPIO25, **RX3** (D15) ← GPIO26; use 5 V → 3.3 V on Mega TX → ESP32 RX.

Change `MEGA_SER_RX_PIN` / `MEGA_SER_TX_PIN` only if you use different free GPIOs (avoid TFT/SPI pins: 2, 4, 15, 18, 19, 23).

## Dashboard buttons (not on the ESP32)

Tact switches live on the **gateway** (Arduino Mega `mega_epic_canbus.ino` or STM32 `STM32 canbus`). The ESP32 only receives **UART** bytes (`N`/`P`/`O`/`Q`/`S`/`R` and newline-terminated status/lines). Wire buttons to the gateway per its `PINOUT.md` / `pinmap.h`.

## TFT_eSPI Values (already chosen)

- `TFT_MISO = 19`
- `TFT_MOSI = 23`
- `TFT_SCLK = 18`
- `TFT_CS = 15`
- `TFT_DC = 2`
- `TFT_RST = 4`

## Quick Wiring Diagram (ASCII)

```text
ESP32 (NodeMCU-32S)                ST7789V Display
--------------------               ----------------
GND -----------------------------> GND
3V3 -----------------------------> VCC
GPIO18 (SCLK) -------------------> SCL
GPIO23 (MOSI) -------------------> SDA
GPIO15 (CS) ---------------------> CS
GPIO2  (DC) ---------------------> DC
GPIO4  (RST) --------------------> RST
GPIO33 (PWM) ---------------------> BL
GPIO19 (MISO, optional) ---------> SDA-0
```

**Brightness:** connect `BL` to **GPIO33**; open **Menu** (gateway **O**), choose **Brightness**, use **N** (dimmer) / **P** (brighter), **Q** back. Level is stored in NVS (`dash` / `bl`).

