# STM32 CAN gateway (Black F407 + rusEFI EPIC)

Arduino-STM32 firmware: **CAN** ↔ ECU (variable get/set), **local IO**, **USART3** ↔ ESP32 dash, **USART2** ↔ GPS.

## Requirements

- Board: **Black F407VG** (`black_f407vg`), PlatformIO + `framework = arduino`
- CAN: **SN65HVD230** (3.3 V), bitrate set in `platformio.ini` (`CAN_BITRATE_KBPS`)
- ECU CAN ID must match TunerStudio (`RUSEFI_EPIC_ECU_CAN_ID`)

## Build / flash

```bash
pio run -e black_f407vg -t upload
```

USB CDC serial: **115200** (`monitor_speed`). Upload: **DFU** (`upload_protocol = dfu`).

## Behaviour (short)

| Area | Role |
|------|------|
| CAN | Round-robin VAR requests; parse `0x720+id` responses; apply `OUT_SLOW` to GPIO/PWM |
| IO | 16× analog, 16× digital, 4× VSS ISR, cruise buttons → packed into ECU frames per `can_ecu_profile.h` |
| Dash UART | **115200 8N1** `PB10` TX → ESP32 RX, `PB11` RX ← ESP32 TX; telemetry + status + `N`/`P`/`O`/`Q`/`S`/`R` from local buttons |
| GPS UART | `PD5`/`PD6`, auto baud **9600 / 115200 / 38400**; NMEA → dash tags + optional ECU GPS vars |
| Bench | `CAN_ALLOW_BOOT_WITHOUT_BUS=1`: boot without transceiver; **ECU off-bus** throttles heavy CAN TX so dash UART/buttons stay responsive |

## CAN IDs (EPIC)

`ecu = RUSEFI_EPIC_ECU_CAN_ID` (default **1**):

- Request `0x700 + ecu` · Response `0x720 + ecu` · Set `0x780 + ecu`

## Dash → ESP32 protocol

Newline-terminated ASCII. **Status:** `R0/1`, `U0/1`, `K0/1`. **Telemetry:** `F D T L B J X M I C V E` + **`a b c d`** (accel seconds) + **`Y Z H g`** (GPS alive/fix/sats/“age” = s since last UART byte). **Oil** `W*` not sent — ESP32 reads oil locally.

**ESP32 → STM32:** `!n` / `@n` (`n` 0–5) for power/extra bits (see `README_PINMAP.md`).

## Source layout

| Path | Content |
|------|---------|
| `include/pinmap.h` | All MCU pins |
| `include/can_ecu_profile.h` | ECU VAR hashes, button packing |
| `src/main.cpp` | Main loop |
| `src/nmea_parser.cpp` | NMEA: `$GP*` / `$GN*` RMC + GGA |

## Bring-up

1. Flash; confirm USB log (CAN OK or SKIPPED per flags).
2. Match CAN bitrate / ECU ID to tune.
3. Link ESP32 **25↔TX**, **26↔RX**, common GND.
4. Confirm telemetry on display; GPS patch on **U.FL**; test buttons.

**Pins & TunerStudio D-map:** `README_PINMAP.md`.
