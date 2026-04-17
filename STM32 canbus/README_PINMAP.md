# Pinmap — STM32 CAN gateway

**Board / env:** `black_f407vg` · **Source of truth:** `include/pinmap.h`

## CAN (SN65HVD230)

| Signal | Pin | Note |
|--------|-----|------|
| TXD | `PB9` | To transceiver CTX |
| RXD | `PB8` | From transceiver CRX |
| | | 120 Ω termination at bus ends as required |

## UART

**GPS (USART2)** — cross-connect to module TTL:

| MCU | GPS module |
|-----|------------|
| `PD5` TX | RX |
| `PD6` RX | TX |

Baud: firmware cycles **9600 → 115200 → 38400** if no RX bytes. NMEA talkers **GP** and **GN** (RMC/GGA) parsed.

**Dashboard (USART3) ↔ ESP32** — 115200 8N1, common GND:

| STM32 | ESP32 (`ESP32displaystandard`) |
|--------|----------------------------------|
| `PB10` TX | `GPIO25` (Serial2 RX) |
| `PB11` RX | `GPIO26` (Serial2 TX) |

## Inputs — dashboard (active low, pull-up, 40 ms debounce)

| Pin | Short | Long |
|-----|-------|------|
| `PC6` | `N` | `O` (1.0 s) |
| `PC7` | `P` | `Q` (1.0 s) |
| `PC8` | `S` | `R` (2.0 s) |

## Inputs — cruise (active low)

`PC10` · `PC11` · `PC12` → `canButtons18` bits (see `can_ecu_profile.h`).

## VSS (interrupt)

`PB6` FL · `PB7` FR · `PA15` RL · `PB3` RR

## Analog / digital / outputs

- **Analog 16:** `PA0–PA7`, `PB0`, `PB1`, `PC0–PC5`
- **Digital 16:** `PD0–PD4`, `PC13`, `PB12`, `PD7–PD15`
- **Slow GPIO 8:** `PE0–PE7`
- **PWM 10:** `PE8–PE14`, `PA8–PA10`

## EPIC CAN ID (same as `README.md`)

Request `0x700 + id` · Response `0x720 + id` · Set `0x780 + id` · default **id = 1**

## TunerStudio — `MEGA_EPIC_D…` (default `RUSEFI_EPIC_BTNMASK_VIA_D22_D37`)

| Source | TS input |
|--------|----------|
| Cruise 1–3 `PC10–PC12` | `D32–D34` |
| Display `!3/@3` … `!5/@5` | `D35–D37` |
| Display `!0/@0` … `!2/@2` | `D29–D31` |

Disable `RUSEFI_EPIC_BTNMASK_VIA_D22_D37` → mapping changes; see `can_ecu_profile.h`.

## ESP32-side GPIO (display firmware only)

See **`ESP32displaystandard/README_PINMAP.md`** (oil, coolant, brake, buzzer, backlight).
