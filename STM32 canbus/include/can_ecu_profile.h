#pragma once

/**
 * rusEFI MEGA_EPIC / EPIC variable CAN profile for uaEFI BIGFUEL-class firmware.
 *
 * Source of truth for hashes: your bundle `docs/variable_status.json` (must match
 * the exact rusEFI build flashed on the ECU). Verified against uaefiBIGFUEL
 * variable_status.json parse_date 2026-03-06.
 *
 * CAN frame IDs follow the same formula as `mega_epic_canbus.ino`:
 *   request  = 0x700 + ecuCanId
 *   response = 0x720 + ecuCanId
 *   set_var  = 0x780 + ecuCanId
 *
 * Set `RUSEFI_EPIC_ECU_CAN_ID` to the same value as TunerStudio
 * "CAN ID - get_var / function calls" (`ecuCanId`). Original Mega sketch used `1`.
 */
#ifndef RUSEFI_EPIC_ECU_CAN_ID
#define RUSEFI_EPIC_ECU_CAN_ID 1
#endif

#include <stdint.h>

static const uint16_t CAN_ID_VAR_REQUEST = (uint16_t)(0x700u + (unsigned)RUSEFI_EPIC_ECU_CAN_ID);
static const uint16_t CAN_ID_VAR_RESPONSE = (uint16_t)(0x720u + (unsigned)RUSEFI_EPIC_ECU_CAN_ID);
static const uint16_t CAN_ID_VARIABLE_SET = (uint16_t)(0x780u + (unsigned)RUSEFI_EPIC_ECU_CAN_ID);

// MEGA_EPIC_1_A0 .. A15 -> MEGA_EPIC_CANBUS_ADC_* in TS
static const int32_t VAR_HASH_ANALOG[16] = {
    595545759, 595545760, 595545761, 595545762,
    595545763, 595545764, 595545765, 595545766,
    595545767, 595545768, -1821826352, -1821826351,
    -1821826350, -1821826349, -1821826348, -1821826347
};

static const int32_t VAR_HASH_D22_D37 = 2138825443;   // MEGA_EPIC_1_D22_D37 (packed digital)
static const int32_t VAR_HASH_OUT_SLOW = 1430780106;  // MEGA_EPIC_1_OUT_SLOW

static const int32_t VAR_HASH_VSS_FRONT_LEFT = -1645222329;   // canVSSFrontLeft
static const int32_t VAR_HASH_VSS_FRONT_RIGHT = 1549498074;  // canVSSFrontRight
static const int32_t VAR_HASH_VSS_REAR_LEFT = 768443592;     // canVSSRearLeft
static const int32_t VAR_HASH_VSS_REAR_RIGHT = -403905157;   // canVSSRearRight

static const int32_t VAR_HASH_CAN_BUTTONS_18 = 355813839;    // canButtons18
static const int32_t VAR_HASH_CAN_BUTTONS_916 = -1143036666; // canButtons916

/**
 * uaEFI BIGFUEL `docs/variable_status.json`: `canButtons18` has outpc_setter SKIPPED — EPIC set_var
 * does not update that output channel. `MEGA_EPIC_1_D22_D37` has outpc_setter ALLOWED.
 *
 * When 1, bits 0..5 of the same logical mask (cruise PC10-12 + display UART bits 3-5) are merged
 * into bits 10..15 of the uint16 sent as `MEGA_EPIC_1_D22_D37` → TunerStudio **MEGA_EPIC_D32..D37**:
 *   bits 0-2 of mask → D32, D33, D34 (cruise) | bits 3-5 → D35, D36, D37 (display menu).
 * Physical digital inputs at DIGITAL_INPUT_PINS indices 10..15 (PD10..PD15) no longer affect those Epic bits.
 * Set to 0 only if you rely on those six physical digitals and another EPIC path for buttons.
 */
#ifndef RUSEFI_EPIC_BTNMASK_VIA_D22_D37
#define RUSEFI_EPIC_BTNMASK_VIA_D22_D37 1
#endif

/** Bit layout for `canButtons18` (packed uint32 sent as float from gateway). */
#define CANBTN18_CRUISE_1       0u  // PC10, active low
#define CANBTN18_CRUISE_2       1u  // PC11
#define CANBTN18_CRUISE_3       2u  // PC12
#define CANBTN18_DISP_ROT_IDLE  3u  // ESP32 menu "Rotational idle" (!3/@3 on dash UART)
#define CANBTN18_DISP_FLATSHIFT 4u  // "FlatShift" (!4/@4)
#define CANBTN18_DISP_ROLL_LAUNCH 5u  // "Rolling launch" (!5/@5)

/** Power menu: ESP32 ids 0–2 → internal btnMask bits 6–8 → Epic D29–D31 (same latch pattern as extras on bits 3–5). */
#define EPIC_BTNMASK_PWR_LOW     6u   // "Low"    → MEGA_EPIC_D29
#define EPIC_BTNMASK_PWR_MEDIUM  7u   // "Medium" → D30
#define EPIC_BTNMASK_PWR_QUATTRO 8u   // "Quattro"→ D31

static const int32_t VAR_HASH_RPM = 1699696209;              // RPMValue
static const int32_t VAR_HASH_INJECTOR_PW_MS = 681043126;    // actualLastInjection
static const int32_t VAR_HASH_VEHICLE_SPEED_KMH = -1925174695; // vehicleSpeedKph
static const int32_t VAR_HASH_CRUISE_ACTIVE = -450760843;    // cc_engaged

/**
 * rusEFI `output_lookup_generated.cpp` hashes (standard output channel names).
 * Must match the ECU bundle if your fork renamed fields; otherwise same as upstream rusEFI.
 */
static const int32_t VAR_HASH_AFR = -1093429509;       // AFRValue (lambda*14.7 style AFR)
static const int32_t VAR_HASH_MAP_KPA = 1281101952;    // MAPValue (kPa absolute typical)
static const int32_t VAR_HASH_IAT = 81034497;          // intake (deg C)
static const int32_t VAR_HASH_CLT = -746111499;       // coolant (deg C)
static const int32_t VAR_HASH_BARO_KPA = -2066867294; // baroPressure (kPa) — boost gauge vs MAP
static const int32_t VAR_HASH_VBATT = 277722310;       // VBatt (V) — injector dead time for dash fuel math

// GPS variable hashes (compatible with MEGA_EPIC_CANBUS GPS mapping)
static const int32_t VAR_HASH_GPS_HMSD_PACKED = 703958849;      // Hours, minutes, seconds, days
static const int32_t VAR_HASH_GPS_MYQSAT_PACKED = -1519914092;  // Months, years, quality, satellites
static const int32_t VAR_HASH_GPS_ACCURACY = -1489698215;
static const int32_t VAR_HASH_GPS_ALTITUDE = -2100224086;
static const int32_t VAR_HASH_GPS_COURSE = 1842893663;
static const int32_t VAR_HASH_GPS_LATITUDE = 1524934922;
static const int32_t VAR_HASH_GPS_LONGITUDE = -809214087;
static const int32_t VAR_HASH_GPS_SPEED = -1486968225;          // km/h
