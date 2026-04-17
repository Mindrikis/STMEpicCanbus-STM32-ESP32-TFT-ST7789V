#include <Arduino.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "pinmap.h"
#include "can_ecu_profile.h"
#include "nmea_parser.h"

#ifndef CAN_BITRATE_KBPS
#define CAN_BITRATE_KBPS 500u
#endif
/** If 1: CAN init failure does not hang — dash UART + UI data still run (no ECU traffic). */
#ifndef CAN_ALLOW_BOOT_WITHOUT_BUS
#define CAN_ALLOW_BOOT_WITHOUT_BUS 0
#endif

static inline void statusLedSet(bool on)
{
#if CAN_STATUS_LED_ACTIVE_LOW
    digitalWrite(CAN_STATUS_LED_PIN, on ? LOW : HIGH);
#else
    digitalWrite(CAN_STATUS_LED_PIN, on ? HIGH : LOW);
#endif
}

static void statusLedPulse(unsigned count, unsigned onMs, unsigned offMs, unsigned gapMs = 0)
{
    for (unsigned i = 0; i < count; i++) {
        statusLedSet(true);
        delay(onMs);
        statusLedSet(false);
        delay(offMs);
    }
    if (gapMs) {
        delay(gapMs);
    }
}

struct can_frame {
    uint32_t can_id;
    uint8_t can_dlc;
    uint8_t data[8];
};

namespace MCP2515 {
    static const int ERROR_OK = 0;
    static const int ERROR_FAIL = -1;
}

class STM32CanInterface {
public:
    bool ensureAwake()
    {
        CAN1->MCR &= ~CAN_MCR_SLEEP;
        const uint32_t t0 = millis();
        while ((CAN1->MSR & CAN_MSR_SLAK) != 0u) {
            if ((millis() - t0) > 5u) return false;
        }
        return true;
    }

    uint32_t getApb1ClockHz() const
    {
        uint32_t hclk = HAL_RCC_GetHCLKFreq();
        uint32_t ppre1 = (RCC->CFGR >> RCC_CFGR_PPRE1_Pos) & 0x7u;
        uint32_t div = 1u;
        if (ppre1 >= 4u) {
            static const uint8_t map[4] = {2, 4, 8, 16};
            div = map[ppre1 - 4u];
        }
        return hclk / div;
    }

    bool begin(uint32_t bitrateKbps)
    {
        RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;
        // Enable all GPIO ports this sketch uses (PB8/9 CAN, PC6–12 dash+cruise, PE*, PA8–10 PWM, etc.).
        // Relying only on A/B/D here can leave GPIOC/GPIOE clocks off until first pinMode timing varies by core.
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN |
                        RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOEEN;

        if (CAN_RX_PIN == PB8 && CAN_TX_PIN == PB9) {
            // CAN1 RX/TX on PB8/PB9, AF9
            GPIOB->MODER &= ~((3u << (8u * 2u)) | (3u << (9u * 2u)));
            GPIOB->MODER |=  ((2u << (8u * 2u)) | (2u << (9u * 2u)));
            GPIOB->OSPEEDR |= ((3u << (8u * 2u)) | (3u << (9u * 2u)));
            GPIOB->PUPDR &= ~((3u << (8u * 2u)) | (3u << (9u * 2u)));
            GPIOB->AFR[1] &= ~((0xFu << ((8u - 8u) * 4u)) | (0xFu << ((9u - 8u) * 4u)));
            GPIOB->AFR[1] |=  ((9u << ((8u - 8u) * 4u)) | (9u << ((9u - 8u) * 4u)));
        } else if (CAN_RX_PIN == PD0 && CAN_TX_PIN == PD1) {
            // CAN1 RX/TX on PD0/PD1, AF9
            GPIOD->MODER &= ~((3u << (0u * 2u)) | (3u << (1u * 2u)));
            GPIOD->MODER |=  ((2u << (0u * 2u)) | (2u << (1u * 2u)));
            GPIOD->OSPEEDR |= ((3u << (0u * 2u)) | (3u << (1u * 2u)));
            GPIOD->PUPDR &= ~((3u << (0u * 2u)) | (3u << (1u * 2u)));
            GPIOD->AFR[0] &= ~((0xFu << (0u * 4u)) | (0xFu << (1u * 4u)));
            GPIOD->AFR[0] |=  ((9u << (0u * 4u)) | (9u << (1u * 4u)));
        } else {
            // CAN1 RX/TX on PA11/PA12, AF9
            GPIOA->MODER &= ~((3u << (11u * 2u)) | (3u << (12u * 2u)));
            GPIOA->MODER |=  ((2u << (11u * 2u)) | (2u << (12u * 2u)));
            GPIOA->OSPEEDR |= ((3u << (11u * 2u)) | (3u << (12u * 2u)));
            GPIOA->PUPDR &= ~((3u << (11u * 2u)) | (3u << (12u * 2u)));
            GPIOA->AFR[1] &= ~((0xFu << ((11u - 8u) * 4u)) | (0xFu << ((12u - 8u) * 4u)));
            GPIOA->AFR[1] |=  ((9u << ((11u - 8u) * 4u)) | (9u << ((12u - 8u) * 4u)));
        }

        // Force CAN out of sleep before configuration (if SLAK stays set, TX never completes).
        CAN1->MCR &= ~CAN_MCR_SLEEP;
        {
            const uint32_t tWake = millis();
            while ((CAN1->MSR & CAN_MSR_SLAK) != 0u) {
                if ((millis() - tWake) > 100u) return false;
            }
        }

        CAN1->MCR |= CAN_MCR_INRQ;
        uint32_t start = millis();
        while ((CAN1->MSR & CAN_MSR_INAK) == 0) {
            if ((millis() - start) > 100) return false;
        }

        uint32_t apb1 = getApb1ClockHz();
        uint32_t prescaler = 0u, ts1 = 0u, ts2 = 0u;
        if (apb1 >= 42000000u) { // 42 MHz APB1
            ts1 = 11u; ts2 = 2u;
            if (bitrateKbps == 500u) prescaler = 6u;
            else if (bitrateKbps == 250u) prescaler = 12u;
            else if (bitrateKbps == 125u) prescaler = 24u;
            else if (bitrateKbps == 100u) prescaler = 30u;
        } else { // 36 MHz APB1 (fallback)
            ts1 = 5u; ts2 = 2u;
            if (bitrateKbps == 500u) prescaler = 9u;
            else if (bitrateKbps == 250u) prescaler = 18u;
            else if (bitrateKbps == 125u) prescaler = 36u;
            else if (bitrateKbps == 100u) prescaler = 45u;
        }
        if (prescaler == 0u) return false;

        CAN1->BTR = ((1u - 1u) << 24) | ((ts2 - 1u) << 20) | ((ts1 - 1u) << 16) | ((prescaler - 1u) << 0);
        CAN1->MCR |= CAN_MCR_ABOM;
        CAN1->MCR &= ~CAN_MCR_TTCM;

        CAN1->MCR &= ~CAN_MCR_INRQ;
        start = millis();
        while (CAN1->MSR & CAN_MSR_INAK) {
            if ((millis() - start) > 100) return false;
        }
        return true;
    }

    bool configureExactStdIdFilter(uint16_t stdId)
    {
        uint32_t id = ((uint32_t)(stdId & 0x7FFu)) << 21;
        uint32_t mask = (0x7FFu << 21);
        CAN1->FMR |= CAN_FMR_FINIT;
        CAN1->FA1R &= ~(1u << 0);
        CAN1->FM1R &= ~(1u << 0);
        CAN1->FS1R |=  (1u << 0);
        CAN1->FFA1R &= ~(1u << 0);
        CAN1->sFilterRegister[0].FR1 = id;
        CAN1->sFilterRegister[0].FR2 = mask;
        CAN1->FA1R |=  (1u << 0);
        CAN1->FMR &= ~CAN_FMR_FINIT;
        return true;
    }

    bool configureAcceptAllStdIdFilter()
    {
        CAN1->FMR |= CAN_FMR_FINIT;
        CAN1->FA1R &= ~(1u << 0);
        CAN1->FM1R &= ~(1u << 0);
        CAN1->FS1R |=  (1u << 0);
        CAN1->FFA1R &= ~(1u << 0);
        CAN1->sFilterRegister[0].FR1 = 0u;
        CAN1->sFilterRegister[0].FR2 = 0u; // mask all 0 => don't care on all bits (accept any)
        CAN1->FA1R |=  (1u << 0);
        CAN1->FMR &= ~CAN_FMR_FINIT;
        return true;
    }

    int sendMessage(const struct can_frame* frame)
    {
        if (!frame || frame->can_dlc > 8) return MCP2515::ERROR_FAIL;
        uint32_t tsr = CAN1->TSR;
        uint8_t mb = 0xFF;
        if (tsr & CAN_TSR_TME0) mb = 0;
        else if (tsr & CAN_TSR_TME1) mb = 1;
        else if (tsr & CAN_TSR_TME2) mb = 2;
        if (mb == 0xFF) return MCP2515::ERROR_FAIL;

        CAN_TxMailBox_TypeDef* m = &CAN1->sTxMailBox[mb];
        m->TIR = ((frame->can_id & 0x7FFu) << 21);
        m->TDTR = (frame->can_dlc & 0xFu);
        m->TDLR = ((uint32_t)frame->data[3] << 24) | ((uint32_t)frame->data[2] << 16) |
                  ((uint32_t)frame->data[1] << 8) | (uint32_t)frame->data[0];
        m->TDHR = ((uint32_t)frame->data[7] << 24) | ((uint32_t)frame->data[6] << 16) |
                  ((uint32_t)frame->data[5] << 8) | (uint32_t)frame->data[4];
        m->TIR |= CAN_TI0R_TXRQ;
        return MCP2515::ERROR_OK;
    }

    int sendMessageAndWaitAck(const struct can_frame* frame, bool* acked, uint16_t maxWaitMs = 50)
    {
        if (acked) *acked = false;
        if (!frame || frame->can_dlc > 8) return MCP2515::ERROR_FAIL;
        if (!ensureAwake()) return MCP2515::ERROR_FAIL;

        uint32_t tsr = CAN1->TSR;
        uint8_t mb = 0xFF;
        if (tsr & CAN_TSR_TME0) mb = 0;
        else if (tsr & CAN_TSR_TME1) mb = 1;
        else if (tsr & CAN_TSR_TME2) mb = 2;
        if (mb == 0xFF) return MCP2515::ERROR_FAIL;

        CAN_TxMailBox_TypeDef* m = &CAN1->sTxMailBox[mb];
        m->TIR = ((frame->can_id & 0x7FFu) << 21);
        m->TDTR = (frame->can_dlc & 0xFu);
        m->TDLR = ((uint32_t)frame->data[3] << 24) | ((uint32_t)frame->data[2] << 16) |
                  ((uint32_t)frame->data[1] << 8) | (uint32_t)frame->data[0];
        m->TDHR = ((uint32_t)frame->data[7] << 24) | ((uint32_t)frame->data[6] << 16) |
                  ((uint32_t)frame->data[5] << 8) | (uint32_t)frame->data[4];
        m->TIR |= CAN_TI0R_TXRQ;

        const uint32_t rqcpMask[3] = {CAN_TSR_RQCP0, CAN_TSR_RQCP1, CAN_TSR_RQCP2};
        const uint32_t txokMask[3] = {CAN_TSR_TXOK0, CAN_TSR_TXOK1, CAN_TSR_TXOK2};
        const uint32_t txFailMask[3] = {CAN_TSR_TERR0, CAN_TSR_TERR1, CAN_TSR_TERR2};
        const uint32_t abrqMask[3] = {CAN_TSR_ABRQ0, CAN_TSR_ABRQ1, CAN_TSR_ABRQ2};
        const unsigned long start = millis();
        const uint16_t abortPhaseMs = (maxWaitMs > 20u) ? (uint16_t)(maxWaitMs / 4u) : 5u;
        while ((CAN1->TSR & rqcpMask[mb]) == 0u) {
            if ((millis() - start) > (unsigned long)maxWaitMs) {
                CAN1->TSR |= abrqMask[mb];
                const unsigned long tabort = millis();
                while ((CAN1->TSR & rqcpMask[mb]) == 0u) {
                    if ((millis() - tabort) > (unsigned long)abortPhaseMs) break;
                }
                CAN1->TSR = rqcpMask[mb];
                return MCP2515::ERROR_FAIL;
            }
        }

        uint32_t doneTsr = CAN1->TSR;
        CAN1->TSR = rqcpMask[mb]; // clear request complete flag
        bool ok = ((doneTsr & txokMask[mb]) != 0u) && ((doneTsr & txFailMask[mb]) == 0u);
        if (acked) *acked = ok;
        return ok ? MCP2515::ERROR_OK : MCP2515::ERROR_FAIL;
    }

    int readMessage(struct can_frame* frame)
    {
        if (!frame) return MCP2515::ERROR_FAIL;
        if ((CAN1->RF0R & CAN_RF0R_FMP0_Msk) == 0) return MCP2515::ERROR_FAIL;

        uint32_t rir = CAN1->sFIFOMailBox[0].RIR;
        uint32_t rdtr = CAN1->sFIFOMailBox[0].RDTR;
        uint32_t rdlr = CAN1->sFIFOMailBox[0].RDLR;
        uint32_t rdhr = CAN1->sFIFOMailBox[0].RDHR;
        if (rir & CAN_RI0R_IDE) {
            CAN1->RF0R |= CAN_RF0R_RFOM0;
            return MCP2515::ERROR_FAIL;
        }

        frame->can_id = (rir >> 21) & 0x7FFu;
        frame->can_dlc = rdtr & 0xFu;
        frame->data[0] = (uint8_t)(rdlr & 0xFFu);
        frame->data[1] = (uint8_t)((rdlr >> 8) & 0xFFu);
        frame->data[2] = (uint8_t)((rdlr >> 16) & 0xFFu);
        frame->data[3] = (uint8_t)((rdlr >> 24) & 0xFFu);
        frame->data[4] = (uint8_t)(rdhr & 0xFFu);
        frame->data[5] = (uint8_t)((rdhr >> 8) & 0xFFu);
        frame->data[6] = (uint8_t)((rdhr >> 16) & 0xFFu);
        frame->data[7] = (uint8_t)((rdhr >> 24) & 0xFFu);
        CAN1->RF0R |= CAN_RF0R_RFOM0;
        return MCP2515::ERROR_OK;
    }
};

static STM32CanInterface CAN;
static bool g_can_hw_ok = false;
/** millis() of last CAN `CAN_ID_VAR_RESPONSE` (0x721). */
static unsigned long lastEcuVarResponseMs = 0u;

/** ECU recently answering — enables heavy 200 ms input burst + 25 ms VAR requests + long TX mailbox wait. */
static bool gatewayEcuFreshLink(unsigned long nowMs)
{
    return lastEcuVarResponseMs != 0u && (nowMs - lastEcuVarResponseMs) < 2000u;
}

/** Long mailbox wait only when ECU recently answered; short wait otherwise so loop() stays fast on open bus. */
static uint16_t gatewayCanMailboxWaitMs(void)
{
    return gatewayEcuFreshLink(millis()) ? 50u : 3u;
}

// Use the (uint32_t rx, uint32_t tx) overload with Arduino pin numbers from pinmap.h.
// Do NOT cast to PinName — digital pin indices (e.g. PB10/PB11 on Black F407) are not PinName enums.
static HardwareSerial DashSerial(DASH_UART_RX_PIN, DASH_UART_TX_PIN);
static HardwareSerial GpsSerial(GPS_UART_RX_PIN, GPS_UART_TX_PIN);

static uint32_t reqTxAckOk = 0, reqTxAckFail = 0, reqTxQueueFail = 0, setTxFail = 0, respRx = 0, rxAny = 0;
static unsigned long lastReqMs = 0, lastInputTxMs = 0;

static unsigned long lastStatusLedMs = 0;
static float ecuRpm = 0.0f;
/** ECU wheel / vehicle speed from CAN (`vehicleSpeedKph`). Source of truth for dash math. */
static float ecuSpeedKmh = 0.0f;
/** Speed sent on dash line `V` and used for trip / instant fuel / acceleration — ECU-based; see `updateDashSpeedKmhForDisplay()`. */
static float dashSpeedKmh = 0.0f;
static float ecuInjMs = 0.0f;
static bool ecuCruise = false;
static unsigned long lastDashTelemMs = 0;
static unsigned long lastCanReqIndex = 0;
static float avgFuelLPer100 = 0.0f;
static float tripDistanceKm = 0.0f;
static unsigned long tripStartMs = 0;

/** ESP32 dash lines `a`/`b`/`c`/`d`: seconds for 0–60 / 0–100 / 0–120 / 60–120 km/h; negative = no valid sample yet. */
static const float kAccelStandstillKmh = 1.5f;
static const float kAccelLaunchMinKmh = 2.5f;
static const unsigned long kAccelArmStopMs = 500u;
/** If a launch never completes (e.g. aborted pull), stop treating it as active so the next pull can arm. */
static const unsigned long kAccelRunGiveUpMs = 90000u;

static unsigned long accel_low_since_ms = 0u;
static bool accel_armed = false;
static bool accel_in_run = false;
static unsigned long accel_t0_ms = 0u;
static bool accel_cap_60 = false;
static bool accel_cap_100 = false;
static bool accel_cap_120 = false;
static unsigned long accel_t60_mark_ms = 0u;
static bool accel_cap_60120 = false;
static float accel_prev_speed_kmh = -1.0f;
static float accel_s_0_60 = -1.0f;
static float accel_s_0_100 = -1.0f;
static float accel_s_0_120 = -1.0f;
static float accel_s_60_120 = -1.0f;

static void dashAccelTick(unsigned long now_ms, float sp)
{
    const float prev = (accel_prev_speed_kmh < 0.0f) ? sp : accel_prev_speed_kmh;

    if (sp <= kAccelStandstillKmh) {
        if (accel_low_since_ms == 0u)
            accel_low_since_ms = now_ms;
        if ((now_ms - accel_low_since_ms) >= kAccelArmStopMs) {
            accel_armed = true;
            if (accel_in_run)
                accel_in_run = false;
        }
    } else {
        accel_low_since_ms = 0u;
    }

    if (!accel_in_run && accel_armed && sp > kAccelLaunchMinKmh) {
        accel_in_run = true;
        accel_armed = false;
        accel_t0_ms = now_ms;
        accel_cap_60 = accel_cap_100 = accel_cap_120 = false;
        accel_t60_mark_ms = 0u;
        accel_cap_60120 = false;
    }

    if (accel_in_run) {
        const float dt_s = (float)(now_ms - accel_t0_ms) / 1000.0f;
        if (!accel_cap_60 && sp >= 60.0f) {
            accel_cap_60 = true;
            accel_s_0_60 = dt_s;
        }
        if (!accel_cap_100 && sp >= 100.0f) {
            accel_cap_100 = true;
            accel_s_0_100 = dt_s;
        }
        if (!accel_cap_120 && sp >= 120.0f) {
            accel_cap_120 = true;
            accel_s_0_120 = dt_s;
        }
        if (accel_t60_mark_ms == 0u && prev < 60.0f && sp >= 60.0f)
            accel_t60_mark_ms = now_ms;
        if (!accel_cap_60120 && accel_t60_mark_ms != 0u && sp >= 120.0f) {
            accel_cap_60120 = true;
            accel_s_60_120 = (float)(now_ms - accel_t60_mark_ms) / 1000.0f;
        }
    }

    if (accel_in_run && (now_ms - accel_t0_ms) >= kAccelRunGiveUpMs)
        accel_in_run = false;

    accel_prev_speed_kmh = sp;
}

struct VssChannel {
    volatile uint32_t edgeCount;
    uint32_t lastCount;
    float pps;
};
static VssChannel vss[4] = {};
static unsigned long lastVssCalcMs = 0;

/** Same semantics as `mega_epic_canbus.ino` `DashButtonState` / `dashButtonTick`. */
struct DashButtonState {
    uint32_t pin;
    char cmdShort;
    char cmdLong;
    unsigned long longHoldMs;
    bool lastLevel;
    unsigned long lastChangeMs;
    unsigned long pressStartMs;
    bool longSent;
};

static DashButtonState dashBtnNext  = {DASH_BTN_NEXT_PIN,  'N', 'O', 1000u, false, 0, 0, false};
static DashButtonState dashBtnPrev  = {DASH_BTN_PREV_PIN,  'P', 'Q', 1000u, false, 0, 0, false};
static DashButtonState dashBtnReset = {DASH_BTN_RESET_PIN, 'S', 'R', 2000u, false, 0, 0, false};
/** Bits 3–5: ESP32 extra menu `!3`..`!5` / `@3`..`@5` → same as cruise in btnMask → Epic D35–D37. */
static uint32_t displayCanButtonBits = 0u;
/** Bits 6–8: power output `!0`..`!2` / `@0`..`@2` — same latch rules as extras → Epic D29–D31. */
static uint32_t displayPowerBtnMask = 0u;
/** 0 idle, 1 saw '!', 2 saw '@' — parse pairs without buffering stray N/P/… bytes. */
static uint8_t dashUartBtnState = 0u;
static bool dashFirst = true;
static bool dashLastRpm = false, dashLastCruise = false, dashLastCheck = false;
static unsigned long lastDashStatusMs = 0;
static unsigned long gpsLastRxMs = 0;
static bool gpsAlive = false;
static bool gpsFix = false;
static uint8_t gpsSats = 0;
static GPSData gpsData = {};
static unsigned long lastGpsEcuTxMs = 0;
static uint8_t gpsEcuTxIndex = 0;
static uint32_t gpsRxByteCount = 0;
static unsigned long lastGpsBaudTryMs = 0;
static uint8_t gpsBaudIdx = 0;
static const uint32_t kGpsBaudOptions[] = {9600u, 115200u, 38400u};
static const uint8_t kGpsBaudCount = (uint8_t)(sizeof(kGpsBaudOptions) / sizeof(kGpsBaudOptions[0]));

/** 0 = dash speed is pure ECU VSS. N>0 = each update blends N/1000 toward GPS SOG when GPS looks healthy (see code). */
#ifndef DASH_GPS_SPEED_BLEND_PERMILLE
#define DASH_GPS_SPEED_BLEND_PERMILLE 0
#endif
#ifndef DASH_GPS_SPEED_BLEND_MAX_DELTA_KMH
#define DASH_GPS_SPEED_BLEND_MAX_DELTA_KMH 18.0f
#endif
#ifndef DASH_GPS_SPEED_BLEND_MIN_SATS
#define DASH_GPS_SPEED_BLEND_MIN_SATS 5u
#endif

static void updateDashSpeedKmhForDisplay(void)
{
    float v = ecuSpeedKmh;
#if DASH_GPS_SPEED_BLEND_PERMILLE > 0
    if (gpsAlive && gpsFix && gpsData.dataValid && gpsData.satellites >= DASH_GPS_SPEED_BLEND_MIN_SATS && gpsData.quality > 0) {
        const float g = gpsData.speed;
        if (g >= 0.0f) {
            const float delta = fabsf(g - ecuSpeedKmh);
            if (delta <= DASH_GPS_SPEED_BLEND_MAX_DELTA_KMH) {
                const float alpha = (float)DASH_GPS_SPEED_BLEND_PERMILLE * 0.001f;
                v = ecuSpeedKmh * (1.0f - alpha) + g * alpha;
            }
        }
    }
#endif
    dashSpeedKmh = v;
}

static inline void writeInt32BigEndian(int32_t value, uint8_t* out)
{
    out[0] = (uint8_t)((value >> 24) & 0xFF);
    out[1] = (uint8_t)((value >> 16) & 0xFF);
    out[2] = (uint8_t)((value >> 8) & 0xFF);
    out[3] = (uint8_t)(value & 0xFF);
}

static inline int32_t readInt32BigEndian(const uint8_t* in)
{
    return ((int32_t)in[0] << 24) | ((int32_t)in[1] << 16) | ((int32_t)in[2] << 8) | (int32_t)in[3];
}

static inline void writeFloat32BigEndian(float value, uint8_t* out)
{
    union { float f; uint32_t u; } conv;
    conv.f = value;
    out[0] = (uint8_t)((conv.u >> 24) & 0xFFu);
    out[1] = (uint8_t)((conv.u >> 16) & 0xFFu);
    out[2] = (uint8_t)((conv.u >> 8) & 0xFFu);
    out[3] = (uint8_t)(conv.u & 0xFFu);
}

static inline float readFloat32BigEndian(const uint8_t* in)
{
    union { float f; uint32_t u; } conv;
    conv.u = ((uint32_t)in[0] << 24) | ((uint32_t)in[1] << 16) | ((uint32_t)in[2] << 8) | (uint32_t)in[3];
    return conv.f;
}

/** ESP32: `!n`/`@n` — ids 0–2 power (D29–D31), 3–5 extra features (bits 3–5). */
static void processDashUartCanButtonCommands(void)
{
    while (DashSerial.available() > 0) {
        int ic = DashSerial.read();
        if (ic < 0)
            break;
        const char c = (char)ic;
        if (c == '\r' || c == '\n') {
            dashUartBtnState = 0u;
            continue;
        }
        if (dashUartBtnState == 0u) {
            if (c == '!')
                dashUartBtnState = 1u;
            else if (c == '@')
                dashUartBtnState = 2u;
            continue;
        }
        if (c >= '0' && c <= '9') {
            const uint8_t id = (uint8_t)(c - '0');
            if (id <= 2u) {
                const uint32_t bit = 1u << (EPIC_BTNMASK_PWR_LOW + id);
                if (dashUartBtnState == 1u)
                    displayPowerBtnMask |= bit;
                else
                    displayPowerBtnMask &= ~bit;
            } else if (id >= 3u && id <= 5u) {
                const uint32_t bit = 1u << id;
                if (dashUartBtnState == 1u)
                    displayCanButtonBits |= bit;
                else
                    displayCanButtonBits &= ~bit;
            }
        }
        dashUartBtnState = 0u;
    }
    displayCanButtonBits &= 0x38u;
    displayPowerBtnMask &= (7u << EPIC_BTNMASK_PWR_LOW);
}

static void sendVariableSetFrameFloat(int32_t hash, float value)
{
    can_frame tx = {};
    tx.can_id = CAN_ID_VARIABLE_SET;
    tx.can_dlc = 8;
    writeInt32BigEndian(hash, &tx.data[0]);
    writeFloat32BigEndian(value, &tx.data[4]);
    // Must wait for mailbox completion: bxCAN has only 3 TX mailboxes; fire-and-forget
    // sendMessage() drops all but the first ~3 frames in this 200ms burst (incl. canButtons18).
    bool busOk = false;
    const uint16_t w = gatewayCanMailboxWaitMs();
    int res = CAN.sendMessageAndWaitAck(&tx, &busOk, w);
    if (res != MCP2515::ERROR_OK || !busOk)
        setTxFail++;
}

static void sendVariableSetFrameU32(int32_t hash, uint32_t value)
{
    can_frame tx = {};
    tx.can_id = CAN_ID_VARIABLE_SET;
    tx.can_dlc = 8;
    writeInt32BigEndian(hash, &tx.data[0]);
    tx.data[4] = (uint8_t)((value >> 24) & 0xFFu);
    tx.data[5] = (uint8_t)((value >> 16) & 0xFFu);
    tx.data[6] = (uint8_t)((value >> 8) & 0xFFu);
    tx.data[7] = (uint8_t)(value & 0xFFu);
    bool busOk = false;
    const uint16_t w = gatewayCanMailboxWaitMs();
    int res = CAN.sendMessageAndWaitAck(&tx, &busOk, w);
    if (res != MCP2515::ERROR_OK || !busOk)
        setTxFail++;
}

static void sendVariableRequestFrame(int32_t hash, bool* ackedOut)
{
    can_frame tx = {};
    tx.can_id = CAN_ID_VAR_REQUEST;
    tx.can_dlc = 4;
    writeInt32BigEndian(hash, tx.data);
    bool acked = false;
    const uint16_t w = gatewayCanMailboxWaitMs();
    int sendRes = CAN.sendMessageAndWaitAck(&tx, &acked, w);
    if (sendRes != MCP2515::ERROR_OK) reqTxQueueFail++;
    else if (acked) reqTxAckOk++;
    else reqTxAckFail++;
    if (ackedOut) *ackedOut = acked;
}

static void applyOutSlowBitfield(uint32_t rawBits)
{
    const uint8_t slowBits = (uint8_t)(rawBits & 0xFFu);
    for (uint8_t i = 0; i < 8; i++) {
        digitalWrite(SLOW_GPIO_PINS[i], (slowBits & (1u << i)) ? HIGH : LOW);
    }
    for (uint8_t i = 0; i < 10; i++) {
        const uint8_t bitIndex = i + 8u;
        analogWrite(PWM_OUTPUT_PINS[i], (rawBits & (1u << bitIndex)) ? 255 : 0);
    }
}

static void vssFrontLeftISR()  { if (vss[0].edgeCount < 0xFFFFFFFEu) vss[0].edgeCount++; }
static void vssFrontRightISR() { if (vss[1].edgeCount < 0xFFFFFFFEu) vss[1].edgeCount++; }
static void vssRearLeftISR()   { if (vss[2].edgeCount < 0xFFFFFFFEu) vss[2].edgeCount++; }
static void vssRearRightISR()  { if (vss[3].edgeCount < 0xFFFFFFFEu) vss[3].edgeCount++; }

static void calculateVssRates(unsigned long now)
{
    if (lastVssCalcMs == 0) {
        lastVssCalcMs = now;
        return;
    }
    const unsigned long dtMs = now - lastVssCalcMs;
    if (dtMs < 25u || dtMs > 1000u) return;
    lastVssCalcMs = now;
    const float dt = dtMs / 1000.0f;
    noInterrupts();
    uint32_t counts[4] = {vss[0].edgeCount, vss[1].edgeCount, vss[2].edgeCount, vss[3].edgeCount};
    interrupts();
    for (uint8_t i = 0; i < 4; i++) {
        const uint32_t delta = counts[i] - vss[i].lastCount;
        vss[i].lastCount = counts[i];
        vss[i].pps = delta / dt;
    }
}

static inline uint32_t packGPSHMSD(uint8_t hours, uint8_t minutes, uint8_t seconds, uint8_t days)
{
    return (uint32_t)hours | ((uint32_t)minutes << 8) | ((uint32_t)seconds << 16) | ((uint32_t)days << 24);
}

static inline uint32_t packGPSMYQSAT(uint8_t months, uint8_t years, uint8_t quality, uint8_t satellites)
{
    return (uint32_t)months | ((uint32_t)years << 8) | ((uint32_t)quality << 16) | ((uint32_t)satellites << 24);
}

static void gpsTick(unsigned long now)
{
    const int kMaxBytes = 192;
    for (int i = 0; i < kMaxBytes && GpsSerial.available() > 0; i++) {
        int ic = GpsSerial.read();
        if (ic < 0) break;
        const char c = (char)ic;
        gpsRxByteCount++;
        gpsLastRxMs = now;
        if (nmeaParserProcessChar(c, &gpsData)) {
            gpsFix = gpsData.hasFix;
            gpsSats = gpsData.satellites;
        }
    }

    gpsAlive = (gpsLastRxMs != 0u) && ((now - gpsLastRxMs) < 3000u);
    if (!gpsAlive) {
        gpsFix = false;
        gpsSats = 0;
        gpsData.dataValid = false;
    }

    // Auto-baud fallback for GT-U7 variants/configs (often 9600 or 115200).
    if (!gpsAlive && (now - lastGpsBaudTryMs) > 5000u) {
        lastGpsBaudTryMs = now;
        gpsBaudIdx = (uint8_t)((gpsBaudIdx + 1u) % kGpsBaudCount);
        GpsSerial.begin(kGpsBaudOptions[gpsBaudIdx]);
        Serial.print(F("GPS baud retry -> "));
        Serial.println((unsigned long)kGpsBaudOptions[gpsBaudIdx]);
        nmeaParserInit();
    }
}

/** Feeds GPS (incl. SOG) to the ECU for its own algorithms. Dash trip / fuel / `V` / accel use ECU VSS via `dashSpeedKmh`, not this path. */
static void sendGpsToEcuIfNeeded(unsigned long now)
{
    if (!g_can_hw_ok) return;
    if (!gatewayEcuFreshLink(now)) return;
    if (!gpsAlive || !gpsData.dataValid) return;
    // Send one GPS variable per slot to avoid long blocking bursts that can trigger resets/WDT.
    if ((now - lastGpsEcuTxMs) < 120u) return;
    lastGpsEcuTxMs = now;
    switch (gpsEcuTxIndex) {
        case 0:
            sendVariableSetFrameU32(VAR_HASH_GPS_HMSD_PACKED,
                                    packGPSHMSD(gpsData.hours, gpsData.minutes, gpsData.seconds, gpsData.days));
            break;
        case 1:
            sendVariableSetFrameU32(VAR_HASH_GPS_MYQSAT_PACKED,
                                    packGPSMYQSAT(gpsData.months, gpsData.years, gpsData.quality, gpsData.satellites));
            break;
        case 2: sendVariableSetFrameFloat(VAR_HASH_GPS_ACCURACY, gpsData.accuracy); break;
        case 3: sendVariableSetFrameFloat(VAR_HASH_GPS_ALTITUDE, gpsData.altitude); break;
        case 4: sendVariableSetFrameFloat(VAR_HASH_GPS_COURSE, gpsData.course); break;
        case 5: sendVariableSetFrameFloat(VAR_HASH_GPS_LATITUDE, gpsData.latitude); break;
        case 6: sendVariableSetFrameFloat(VAR_HASH_GPS_LONGITUDE, gpsData.longitude); break;
        default: sendVariableSetFrameFloat(VAR_HASH_GPS_SPEED, gpsData.speed); break;
    }
    gpsEcuTxIndex = (uint8_t)((gpsEcuTxIndex + 1u) % 8u);
}

static void dashButtonTick(DashButtonState& btn, unsigned long nowMs)
{
    const bool pressed = (digitalRead(btn.pin) == LOW);
    if (pressed != btn.lastLevel) {
        btn.lastLevel = pressed;
        btn.lastChangeMs = nowMs;
    }
    if ((nowMs - btn.lastChangeMs) < 40u)
        return;
    if (btn.lastLevel) {
        if (btn.pressStartMs == 0u) {
            btn.pressStartMs = nowMs;
            btn.longSent = false;
        } else if (!btn.longSent && (nowMs - btn.pressStartMs) >= btn.longHoldMs) {
            DashSerial.write((uint8_t)btn.cmdLong);
            DashSerial.flush();
            btn.longSent = true;
        }
    } else {
        if (btn.pressStartMs != 0u) {
            if (!btn.longSent) {
                DashSerial.write((uint8_t)btn.cmdShort);
                DashSerial.flush();
            }
            btn.pressStartMs = 0u;
            btn.longSent = false;
        }
    }
}

static void sendDashStatus(unsigned long now)
{
    const bool rpmDetected = ecuRpm > 200.0f;
    const bool cruiseOn = ecuCruise;
    const bool checkWarning = !rpmDetected;
    if (dashFirst) {
        dashFirst = false;
        DashSerial.print(rpmDetected ? "R1\n" : "R0\n");
        DashSerial.print(cruiseOn ? "U1\n" : "U0\n");
        DashSerial.print(checkWarning ? "K1\n" : "K0\n");
        DashSerial.flush();
        dashLastRpm = rpmDetected; dashLastCruise = cruiseOn; dashLastCheck = checkWarning;
        lastDashStatusMs = now;
        return;
    }
    if ((now - lastDashStatusMs) < 150u) return;
    lastDashStatusMs = now;
    if (rpmDetected != dashLastRpm) { dashLastRpm = rpmDetected; DashSerial.print(rpmDetected ? "R1\n" : "R0\n"); }
    if (cruiseOn != dashLastCruise) { dashLastCruise = cruiseOn; DashSerial.print(cruiseOn ? "U1\n" : "U0\n"); }
    if (checkWarning != dashLastCheck) { dashLastCheck = checkWarning; DashSerial.print(checkWarning ? "K1\n" : "K0\n"); }
    DashSerial.flush();
}

static void sendDashTelemetry(unsigned long now)
{
    if (tripStartMs == 0u) tripStartMs = now;
    if ((now - lastDashTelemMs) < 200u) return; // ~5Hz
    const unsigned long prev_ms = lastDashTelemMs;
    const float dtHours = (prev_ms == 0u) ? 0.0f : ((float)(now - prev_ms) / 3600000.0f);
    lastDashTelemMs = now;

    // Basic derived signals for ESP32 display fields (speed = ECU VSS, optionally lightly blended toward GPS SOG; see updateDashSpeedKmhForDisplay).
    tripDistanceKm += dashSpeedKmh * dtHours;
    const float speed = dashSpeedKmh;
    const float fuelLph = ecuInjMs * ecuRpm * 0.00045f; // rough synthetic conversion
    float instL100 = (speed > 1.0f) ? (fuelLph / speed) * 100.0f : 0.0f;
    if (instL100 < 0.0f) instL100 = 0.0f;
    if (instL100 > 60.0f) instL100 = 60.0f;
    if (avgFuelLPer100 <= 0.01f) avgFuelLPer100 = instL100;
    else avgFuelLPer100 = avgFuelLPer100 * 0.97f + instL100 * 0.03f;

    const float load = (ecuRpm / 7000.0f);
    const float boostBar = (load * 1.7f) - 0.7f; // approx -0.7 .. +1.0
    const float afr = 14.7f - load * 2.6f;
    const float mapKpa = 28.0f + load * 140.0f;
    const float iatC = 28.0f + load * 20.0f;
    const float cltC = 75.0f + load * 15.0f;
    const float tripMin = (now - tripStartMs) / 60000.0f;

    DashSerial.print("F"); DashSerial.println(instL100, 1);
    DashSerial.print("D"); DashSerial.println(tripDistanceKm, 1);
    DashSerial.print("T"); DashSerial.println(tripMin, 0);
    DashSerial.print("L"); DashSerial.println(avgFuelLPer100, 1);
    DashSerial.print("B"); DashSerial.println(boostBar, 1);
    DashSerial.print("J"); DashSerial.println(ecuInjMs, 2);
    DashSerial.print("X"); DashSerial.println(afr, 2);
    DashSerial.print("M"); DashSerial.println(mapKpa, 0);
    DashSerial.print("I"); DashSerial.println(iatC, 0);
    DashSerial.print("C"); DashSerial.println(cltC, 0);
    DashSerial.print("V"); DashSerial.println(dashSpeedKmh, 0);
    DashSerial.print("E"); DashSerial.println(ecuRpm, 0);
    const float gpsAgeSec = (gpsLastRxMs == 0u) ? 999.0f : ((now - gpsLastRxMs) / 1000.0f);
    DashSerial.print("Y"); DashSerial.println(gpsAlive ? 1 : 0);
    DashSerial.print("Z"); DashSerial.println(gpsFix ? 1 : 0);
    DashSerial.print("H"); DashSerial.println((int)gpsSats);
    DashSerial.print("g"); DashSerial.println(gpsAgeSec, 1);
    DashSerial.print("a"); DashSerial.println(accel_s_0_60 >= 0.0f ? accel_s_0_60 : -1.0f, 2);
    DashSerial.print("b"); DashSerial.println(accel_s_0_100 >= 0.0f ? accel_s_0_100 : -1.0f, 2);
    DashSerial.print("c"); DashSerial.println(accel_s_0_120 >= 0.0f ? accel_s_0_120 : -1.0f, 2);
    DashSerial.print("d"); DashSerial.println(accel_s_60_120 >= 0.0f ? accel_s_60_120 : -1.0f, 2);
    DashSerial.flush();
}

void setup()
{
    Serial.begin(115200);
    delay(150);
    Serial.println("STM32 CAN bridge boot");

    pinMode(PA1, OUTPUT); // human-visible link-status LED mirror
    digitalWrite(PA1, LOW);
    pinMode(CAN_STATUS_LED_PIN, OUTPUT);
    statusLedSet(false);

    delay(80);
    // Boot marker: two short blinks = firmware running
    statusLedPulse(2, 60, 100, 200);

    g_can_hw_ok = CAN.begin(CAN_BITRATE_KBPS);
    if (g_can_hw_ok)
        CAN.configureAcceptAllStdIdFilter();
    if (!g_can_hw_ok) {
        Serial.println("CAN init FAIL (no transceiver, wrong pins, or bxCAN error)");
#if CAN_ALLOW_BOOT_WITHOUT_BUS
        Serial.println("CAN_ALLOW_BOOT_WITHOUT_BUS=1: starting dash/UART anyway — fix CAN for ECU link.");
#else
        Serial.println("Build with -D CAN_ALLOW_BOOT_WITHOUT_BUS=1 to run without CAN for bench/display test.");
        for (;;) {
            statusLedPulse(1, 400, 600, 0);
            delay(400);
        }
#endif
    }

    Serial.print("CAN init ");
    Serial.print(g_can_hw_ok ? "OK" : "SKIPPED");
    Serial.print(" @");
    Serial.print((unsigned long)CAN_BITRATE_KBPS);
    Serial.println(" kbps");
    Serial.print("CAN pins RX/TX: ");
    Serial.print((int)CAN_RX_PIN);
    Serial.print("/");
    Serial.println((int)CAN_TX_PIN);
    Serial.print("EPIC ecuCanId=");
    Serial.print(RUSEFI_EPIC_ECU_CAN_ID);
    Serial.print(" CAN req/rsp/set 0x");
    Serial.print((unsigned)CAN_ID_VAR_REQUEST, HEX);
    Serial.print("/0x");
    Serial.print((unsigned)CAN_ID_VAR_RESPONSE, HEX);
    Serial.print("/0x");
    Serial.println((unsigned)CAN_ID_VARIABLE_SET, HEX);

    // Configure gateway IO to mirror mega_epic_canbus behavior.
    for (uint8_t i = 0; i < 16; i++) {
        pinMode(ANALOG_INPUT_PINS[i], INPUT);
    }
    for (uint8_t i = 0; i < 16; i++) {
        pinMode(DIGITAL_INPUT_PINS[i], INPUT_PULLUP);
    }
    for (uint8_t i = 0; i < 8; i++) {
        pinMode(SLOW_GPIO_PINS[i], OUTPUT);
        digitalWrite(SLOW_GPIO_PINS[i], LOW);
    }
    for (uint8_t i = 0; i < 10; i++) {
        pinMode(PWM_OUTPUT_PINS[i], OUTPUT);
        analogWrite(PWM_OUTPUT_PINS[i], 0);
    }
    pinMode(VSS_FRONT_LEFT_PIN, INPUT_PULLUP);
    pinMode(VSS_FRONT_RIGHT_PIN, INPUT_PULLUP);
    pinMode(VSS_REAR_LEFT_PIN, INPUT_PULLUP);
    pinMode(VSS_REAR_RIGHT_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(VSS_FRONT_LEFT_PIN), vssFrontLeftISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(VSS_FRONT_RIGHT_PIN), vssFrontRightISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(VSS_REAR_LEFT_PIN), vssRearLeftISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(VSS_REAR_RIGHT_PIN), vssRearRightISR, FALLING);
    pinMode(DASH_BTN_NEXT_PIN, INPUT_PULLUP);
    pinMode(DASH_BTN_PREV_PIN, INPUT_PULLUP);
    pinMode(DASH_BTN_RESET_PIN, INPUT_PULLUP);
    pinMode(CRUISE_BTN_1_PIN, INPUT_PULLUP);
    pinMode(CRUISE_BTN_2_PIN, INPUT_PULLUP);
    pinMode(CRUISE_BTN_3_PIN, INPUT_PULLUP);
    nmeaParserInit();
    gpsBaudIdx = 0;
    GpsSerial.begin(kGpsBaudOptions[gpsBaudIdx]);
    lastGpsBaudTryMs = millis();
    Serial.print(F("GPS UART USART2: RX="));
    Serial.print((int)GPS_UART_RX_PIN);
    Serial.print(F(" TX="));
    Serial.print((int)GPS_UART_TX_PIN);
    Serial.print(F(" @"));
    Serial.println((unsigned long)kGpsBaudOptions[gpsBaudIdx]);
    DashSerial.begin(115200);
    Serial.println(F("Dash UART USART3: RX=PB11 TX=PB10 @115200 — TX->ESP32 GPIO25, RX<-ESP32 GPIO26, GND common"));

    // One medium blink = CAN+BTR OK, accept-all filter on — ready for transceiver + bus
    statusLedPulse(1, 120, 150, 0);
}

void loop()
{
    unsigned long now = millis();
    gpsTick(now);
    sendGpsToEcuIfNeeded(now);

    // CAN_STATUS_LED_PIN: runtime patterns (no separate heartbeat — blink patterns = alive)
    if (now - lastStatusLedMs >= 50) {
        lastStatusLedMs = now;
        const bool haveResponse = (respRx > 0);

        if (!haveResponse) {
            // Waiting for first 0x721: brief flash every ~2 s
            const unsigned phase = (now % 2000u);
            statusLedSet(phase < 80u);
        } else {
            // At least one 0x721 received: faster double-blink every ~1.5 s
            const unsigned p = (now % 1500u);
            const bool d = (p < 60u) || (p >= 200u && p < 260u);
            statusLedSet(d);
        }
    }

    // PA1 mirror: easier-to-see status than the previous 5 ms strobe.
    if (respRx == 0) {
        // No response yet: one short blink every 1 s.
        const unsigned p = (now % 1000u);
        digitalWrite(PA1, (p < 120u) ? HIGH : LOW);
    } else {
        // Response seen: double-blink every 1.2 s.
        const unsigned p = (now % 1200u);
        const bool on = (p < 120u) || (p >= 240u && p < 360u);
        digitalWrite(PA1, on ? HIGH : LOW);
    }

    unsigned reqGapMs;
    if (gatewayEcuFreshLink(now))
        reqGapMs = 25u;
    else if (lastEcuVarResponseMs == 0u)
        reqGapMs = (now < 10000u) ? 100u : 2000u; /* never saw ECU: fast probe then rare */
    else
        reqGapMs = 500u; /* had ECU, link stale: slow probe until reconnect */

    if (g_can_hw_ok && (now - lastReqMs >= reqGapMs)) {
        lastReqMs = now;
        // Round-robin ECU requests used by gateway + display.
        switch (lastCanReqIndex % 5u) {
            case 0: sendVariableRequestFrame(VAR_HASH_OUT_SLOW, nullptr); break;
            case 1: sendVariableRequestFrame(VAR_HASH_RPM, nullptr); break;
            case 2: sendVariableRequestFrame(VAR_HASH_INJECTOR_PW_MS, nullptr); break;
            case 3: sendVariableRequestFrame(VAR_HASH_VEHICLE_SPEED_KMH, nullptr); break;
            default: sendVariableRequestFrame(VAR_HASH_CRUISE_ACTIVE, nullptr); break;
        }
        lastCanReqIndex++;
    }

    calculateVssRates(now);

    processDashUartCanButtonCommands();

    if (g_can_hw_ok && gatewayEcuFreshLink(now) && (now - lastInputTxMs >= 200)) {
        lastInputTxMs = now;
        uint32_t btnMask = 0u;
        if (digitalRead(CRUISE_BTN_1_PIN) == LOW) btnMask |= 1u << CANBTN18_CRUISE_1;
        if (digitalRead(CRUISE_BTN_2_PIN) == LOW) btnMask |= 1u << CANBTN18_CRUISE_2;
        if (digitalRead(CRUISE_BTN_3_PIN) == LOW) btnMask |= 1u << CANBTN18_CRUISE_3;
        btnMask |= displayCanButtonBits;
        btnMask |= displayPowerBtnMask;
        // Send button state before the long analog burst so ECU always sees it even if the bus/node is busy.
        sendVariableSetFrameFloat(VAR_HASH_CAN_BUTTONS_18, (float)(btnMask & 0xFFu));
        sendVariableSetFrameFloat(VAR_HASH_CAN_BUTTONS_916, 0.0f);
        for (uint8_t i = 0; i < 16; i++) {
            const float adc = (float)analogRead(ANALOG_INPUT_PINS[i]);
            sendVariableSetFrameFloat(VAR_HASH_ANALOG[i], adc);
        }
        uint16_t digitalBits = 0;
        for (uint8_t i = 0; i < 16; i++) {
            // Active-low inputs: grounded -> bit set.
            if (digitalRead(DIGITAL_INPUT_PINS[i]) == LOW) {
                digitalBits |= (uint16_t)(1u << i);
            }
        }
#if RUSEFI_EPIC_BTNMASK_VIA_D22_D37
        // canButtons18 setter SKIPPED on uaEFI BIGFUEL; pack btnMask into MEGA_EPIC_1_D22_D37 like extras (D35–D37).
        digitalBits &= (uint16_t)~(((uint16_t)0x3Fu << 10) | 0x380u);
        digitalBits |= (uint16_t)((btnMask & 0x3Fu) << 10);
        if (btnMask & (1u << EPIC_BTNMASK_PWR_LOW))
            digitalBits |= (1u << 7);
        if (btnMask & (1u << EPIC_BTNMASK_PWR_MEDIUM))
            digitalBits |= (1u << 8);
        if (btnMask & (1u << EPIC_BTNMASK_PWR_QUATTRO))
            digitalBits |= (1u << 9);
#else
        digitalBits &= (uint16_t)~0x380u;
        if (btnMask & (1u << EPIC_BTNMASK_PWR_LOW))
            digitalBits |= (1u << 7);
        if (btnMask & (1u << EPIC_BTNMASK_PWR_MEDIUM))
            digitalBits |= (1u << 8);
        if (btnMask & (1u << EPIC_BTNMASK_PWR_QUATTRO))
            digitalBits |= (1u << 9);
#endif
        sendVariableSetFrameFloat(VAR_HASH_D22_D37, (float)digitalBits);
        sendVariableSetFrameFloat(VAR_HASH_VSS_FRONT_LEFT, vss[0].pps);
        sendVariableSetFrameFloat(VAR_HASH_VSS_FRONT_RIGHT, vss[1].pps);
        sendVariableSetFrameFloat(VAR_HASH_VSS_REAR_LEFT, vss[2].pps);
        sendVariableSetFrameFloat(VAR_HASH_VSS_REAR_RIGHT, vss[3].pps);
    }

    can_frame rx = {};
    while (g_can_hw_ok && (CAN.readMessage(&rx) == MCP2515::ERROR_OK)) {
        rxAny++;
        if (rx.can_id == CAN_ID_VAR_RESPONSE && rx.can_dlc == 8) {
            respRx++;
            lastEcuVarResponseMs = millis();
            const int32_t hash = readInt32BigEndian(&rx.data[0]);
            if (hash == VAR_HASH_OUT_SLOW) {
                float value = readFloat32BigEndian(&rx.data[4]);
                const uint32_t rawBits = (value >= 0.0f) ? (uint32_t)(value + 0.5f) : 0u;
                applyOutSlowBitfield(rawBits);
            } else if (hash == VAR_HASH_RPM) {
                ecuRpm = readFloat32BigEndian(&rx.data[4]);
            } else if (hash == VAR_HASH_INJECTOR_PW_MS) {
                ecuInjMs = readFloat32BigEndian(&rx.data[4]);
            } else if (hash == VAR_HASH_VEHICLE_SPEED_KMH) {
                ecuSpeedKmh = readFloat32BigEndian(&rx.data[4]);
            } else if (hash == VAR_HASH_CRUISE_ACTIVE) {
                ecuCruise = readFloat32BigEndian(&rx.data[4]) > 0.5f;
            }
        }
    }

    updateDashSpeedKmhForDisplay();
    dashAccelTick(now, dashSpeedKmh);

    dashButtonTick(dashBtnNext, now);
    dashButtonTick(dashBtnPrev, now);
    dashButtonTick(dashBtnReset, now);
    sendDashStatus(now);
    sendDashTelemetry(now);

    static unsigned long lastPrint = 0;
    if (now - lastPrint >= 1000u) {
        lastPrint = now;
        if (!g_can_hw_ok) {
            Serial.println("CAN off (dash-only mode)");
        } else {
            const uint32_t esr = CAN1->ESR;
            // 1 = button pressed (active-low, pin shorted to GND)
            const int c1 = (digitalRead(CRUISE_BTN_1_PIN) == LOW) ? 1 : 0;
            const int c2 = (digitalRead(CRUISE_BTN_2_PIN) == LOW) ? 1 : 0;
            const int c3 = (digitalRead(CRUISE_BTN_3_PIN) == LOW) ? 1 : 0;
            const uint32_t cm = ((uint32_t)c1 << CANBTN18_CRUISE_1) | ((uint32_t)c2 << CANBTN18_CRUISE_2) |
                                ((uint32_t)c3 << CANBTN18_CRUISE_3) | displayCanButtonBits | displayPowerBtnMask;
            Serial.print("txAckOk=");
            Serial.print(reqTxAckOk);
            Serial.print(" txAckFail=");
            Serial.print(reqTxAckFail);
            Serial.print(" txQueueFail=");
            Serial.print(reqTxQueueFail);
            Serial.print(" setTxFail=");
            Serial.print(setTxFail);
            Serial.print(" rxAny=");
            Serial.print(rxAny);
            Serial.print(" respRx=");
            Serial.print(respRx);
            Serial.print(" TEC=");
            Serial.print((esr >> 16) & 0xFFu);
            Serial.print(" REC=");
            Serial.print((esr >> 24) & 0xFFu);
            Serial.print(" cruise123=");
            Serial.print(c1);
            Serial.print(c2);
            Serial.print(c3);
            Serial.print(" canBtn18mask=");
            Serial.print(cm);
            Serial.print(" pwrBtn678=");
            Serial.print((unsigned)((displayPowerBtnMask >> EPIC_BTNMASK_PWR_LOW) & 7u));
            Serial.print(" gpsAlive=");
            Serial.print(gpsAlive ? 1 : 0);
            Serial.print(" gpsFix=");
            Serial.print(gpsFix ? 1 : 0);
            Serial.print(" gpsSat=");
            Serial.print((unsigned)gpsSats);
            Serial.print(" gpsRxBytes=");
            Serial.println((unsigned long)gpsRxByteCount);
        }
    }
}