#pragma once

#include <Arduino.h>

// ---------------- CAN transceiver pins (SN65HVD230) ----------------
static const uint8_t CAN_RX_PIN = PB8;    // CAN1_RX
static const uint8_t CAN_TX_PIN = PB9;    // CAN1_TX

// Bring-up / status LED (many boards have no PC13 LED — wire LED+resistor here or pick another free GPIO)
static const uint32_t CAN_STATUS_LED_PIN = PA6;
// 1 = active-low (cathode to MCU, anode to Vdd via resistor); 0 = active-high
#ifndef CAN_STATUS_LED_ACTIVE_LOW
#define CAN_STATUS_LED_ACTIVE_LOW 0
#endif

// ---------------- Analog input channels (16) ----------------
static const uint32_t ANALOG_INPUT_PINS[16] = {
    PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7,
    PB0, PB1, PC0, PC1, PC2, PC3, PC4, PC5
};

// ---------------- Digital input channels (16) ----------------
static const uint32_t DIGITAL_INPUT_PINS[16] = {
    PD0, PD1, PD2, PD3, PD4, PC13, PB12, PD7,
    PD8, PD9, PD10, PD11, PD12, PD13, PD14, PD15
};

// ---------------- Output channels ----------------
static const uint32_t SLOW_GPIO_PINS[8] = {PE0, PE1, PE2, PE3, PE4, PE5, PE6, PE7};
static const uint32_t PWM_OUTPUT_PINS[10] = {PE8, PE9, PE10, PE11, PE12, PE13, PE14, PA8, PA9, PA10};

// ---------------- VSS interrupt pins ----------------
static const uint32_t VSS_FRONT_LEFT_PIN  = PB6;
static const uint32_t VSS_FRONT_RIGHT_PIN = PB7;
static const uint32_t VSS_REAR_LEFT_PIN   = PA15;
static const uint32_t VSS_REAR_RIGHT_PIN  = PB3;

// ---------------- UART pin mapping ----------------
static const uint32_t GPS_UART_RX_PIN = PD6;    // USART2_RX
static const uint32_t GPS_UART_TX_PIN = PD5;    // USART2_TX
static const uint32_t DASH_UART_RX_PIN = PB11;  // USART3_RX
static const uint32_t DASH_UART_TX_PIN = PB10;  // USART3_TX

// ---------------- Dashboard button/status pins ----------------
static const uint32_t DASH_BTN_NEXT_PIN  = PC6;
static const uint32_t DASH_BTN_PREV_PIN  = PC7;
static const uint32_t DASH_BTN_RESET_PIN = PC8;

// ---------------- Cruise control button pins (active-low) ----------------
static const uint32_t CRUISE_BTN_1_PIN = PC10;
static const uint32_t CRUISE_BTN_2_PIN = PC11;
static const uint32_t CRUISE_BTN_3_PIN = PC12;
