/**
 * ESP32 + ST7789V TFT (240x320) + LVGL dashboard using TFT_eSPI
 *
 * This is adapted from an Arduino Nano ESP32 + ILI9341 + Adafruit_ILI9341
 * version. All UI logic and LVGL widgets are preserved, only the display
 * driver / wiring layer is changed to use TFT_eSPI with an ST7789V panel.
 *
 * IMPORTANT:
 *   - Configure your pins and driver in TFT_eSPI's User_Setup:
 *       - Set driver to ST7789_2 or similar ST7789 variant
 *       - Set TFT_WIDTH  to 240, TFT_HEIGHT to 320
 *       - Set your SPI pins and CS / DC / RST etc.
 *   - This sketch assumes portrait 240x320 orientation.
 */

#include <Arduino.h>
#include <SPI.h>
#include <Preferences.h>
#include <TFT_eSPI.h>
#include <lvgl.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#if ARDUINO_ARCH_ESP32 && defined(CONFIG_BT_ENABLED) && CONFIG_BT_ENABLED
#include "esp_bt.h"
#endif

// TFT_eSPI manages TFT pins and SPI frequency via its own config.
// LVGL logical resolution must match the configured TFT size.
#define TFT_WIDTH   240
#define TFT_HEIGHT  320

// Trip-computer UART: STM32F407 gateway <-> ESP32 Serial2 on fixed GPIOs (NOT TX0/RX0).
// Wiring STM32 (see STM32 canbus README_PINMAP.md / pinmap.h DASH_UART_*):
//   STM32 PB10 (TX) -> ESP32 GPIO25 (Serial2 RX)
//   STM32 PB11 (RX) <- ESP32 GPIO26 (Serial2 TX)
//   GND <-> GND
// Legacy Mega wiring was: Mega TX3 (D14)->GPIO25, Mega RX3 (D15)<-GPIO26 (same cross-TX/RX rule).
// Using UART2 keeps UART0 free for USB upload / serial monitor.
#define MEGA_BAUD        115200
#define MEGA_SER_RX_PIN  25  // STM32 PB10 (TX) -> here (Serial2 RX)
#define MEGA_SER_TX_PIN  26  // -> STM32 PB11 (RX) (Serial2 TX)

// Oil pressure on ESP32 (active-low = OK). Override with build flag if needed: `-D OIL_PRESSURE_SWITCH_PIN=32`
#ifndef OIL_PRESSURE_SWITCH_PIN
#define OIL_PRESSURE_SWITCH_PIN  27
#endif
/* Coolant / brake: INPUT_PULLUP — open = HIGH = OK; switch/sensor pulls to GND when level low → warn. */
#ifndef COOLANT_LEVEL_PIN
#define COOLANT_LEVEL_PIN  13
#endif
#ifndef BRAKE_FLUID_PIN
#define BRAKE_FLUID_PIN  14
#endif
/** Coolant + brake fluid warning strip and buzzer chirps. Set via `-D DASH_FLUID_LEVEL_WARNINGS=0` in platformio.ini to disable. */
#ifndef DASH_FLUID_LEVEL_WARNINGS
#define DASH_FLUID_LEVEL_WARNINGS  1
#endif
/** Oil / fluid pins must read “fault” continuously this long (ms) before overlay + buzzer — avoids glitches from loose wires. */
#ifndef DASH_SENSOR_FAULT_CONFIRM_MS
#define DASH_SENSOR_FAULT_CONFIRM_MS  2000u
#endif
// Avoid GPIO 5 (VSPI SS); default matches prior working wiring.
#ifndef BUZZER_PIN
#define BUZZER_PIN  16
#endif
/** TFT backlight PWM: move display `BL` from 3V3 to this pin (active high). Override: `-D TFT_BACKLIGHT_PIN=32`. */
#ifndef TFT_BACKLIGHT_PIN
#define TFT_BACKLIGHT_PIN  33
#endif
#ifndef BACKLIGHT_PWM_HZ
#define BACKLIGHT_PWM_HZ  5000u
#endif
#ifndef BACKLIGHT_PWM_BITS
#define BACKLIGHT_PWM_BITS  8u
#endif
/** Minimum on-screen % (avoid fully off). */
#ifndef BACKLIGHT_MIN_PCT
#define BACKLIGHT_MIN_PCT  8u
#endif
#ifndef BACKLIGHT_STEP_PCT
#define BACKLIGHT_STEP_PCT  5u
#endif
#ifndef BUZZER_ACTIVE_HIGH
#define BUZZER_ACTIVE_HIGH  1
#endif

// Gateway (Mega or STM32) -> ESP32 Serial2: N/P navigate; O/Q menu; S / R = trip reset (bare R is disambiguated from R0/R1 status).
// Buttons are GPIO on the gateway only — not on the ESP32.
// ESP32 -> Mega lines: `!0`..`!5` activate epic id; `@0`..`@5` deactivate (power change sends @old then !new).
#define CMD_NEXT_PAGE       'N'
#define CMD_PREV_PAGE       'P'
#define CMD_RESET_TRIP      'R'
#define CMD_OPEN_PWR_OUTPUT 'O'
#define CMD_QUIT_PWR_OUTPUT 'Q'
#define CMD_SELECT_PWR      'S' /* optional alias for R on leaf menus */

/** Set to 1 (e.g. build flag `-D DASH_LINK_SNIFFER=1`) to mirror raw Serial2 RX bytes to USB Serial for wiring checks. */
#ifndef DASH_LINK_SNIFFER
#define DASH_LINK_SNIFFER 0
#endif

// Colors and layout constants kept from original code
#define COLOR_BG       lv_color_hex(0x404040)
#define COLOR_CARD     lv_color_hex(0x4C4C4C)
#define COLOR_TEXT     lv_color_hex(0xE8E8E8)
#define COLOR_ACCENT   lv_color_hex(0xD0D0D0)
/** Menu row: focused / “activated” selection (grey letters) */
#define COLOR_MENU_ACTIVE_GREY  lv_color_hex(0x9A9A9A)

#define SPLASH_DURATION_MS   3000
#define SPLASH_HANDLER_LOOPS 4
#define BOTTOM_BAR_H         34
/** Raise power-tab sensor pills (~5 mm on 320px-tall panel) so bottom indicator pop-ups don’t cover them */
#define PWR_GRID_RAISE_PX    38
#define NUM_TABS             11
#define NUM_MENU_ROOT_OPTS   4
#define NUM_PWR_OUT_OPTS     3
#define NUM_EXTRA_OPTS       3
#define NUM_LANG_OPTS        2

typedef enum {
  MENU_LAYER_NONE = 0,
  MENU_LAYER_ROOT,
  MENU_LAYER_PWR,
  MENU_LAYER_EXTRA,
  MENU_LAYER_BRIGHTNESS,
  MENU_LAYER_LANGUAGE
} menu_layer_t;

typedef enum {
  LANG_EN = 0,
  LANG_LV = 1
} ui_lang_t;

/* Partial buffers (~19 KB for two stripes @ 20 lines). */
#define LVGL_BUF_LINES 20
static uint16_t buf1[TFT_WIDTH * LVGL_BUF_LINES];
static uint16_t buf2[TFT_WIDTH * LVGL_BUF_LINES];
static lv_display_t *display = nullptr;

// Global TFT_eSPI display instance
static TFT_eSPI tft = TFT_eSPI();

static lv_obj_t *splash_cont = NULL;
/** Full-screen holder for dashboard pages (not lv_tabview — manual show/hide per tab). */
static lv_obj_t *main_tabview = NULL;
static lv_obj_t *bottom_bar = NULL;
static bool splash_finished = false;

static lv_obj_t *menu_root_overlay = NULL;
static lv_obj_t *menu_root_arrows[NUM_MENU_ROOT_OPTS] = { NULL };
static lv_obj_t *menu_root_opt_lbls[NUM_MENU_ROOT_OPTS] = { NULL };
static uint8_t menu_root_sel = 0;

static lv_obj_t *pwr_out_overlay = NULL;
static lv_obj_t *pwr_out_arrows[NUM_PWR_OUT_OPTS] = { NULL };
static lv_obj_t *pwr_out_opt_lbls[NUM_PWR_OUT_OPTS] = { NULL };
static uint8_t pwr_out_selection = 0;

static lv_obj_t *extra_overlay = NULL;
static lv_obj_t *extra_arrows[NUM_EXTRA_OPTS] = { NULL };
static lv_obj_t *extra_opt_lbls[NUM_EXTRA_OPTS] = { NULL };
static uint8_t menu_extra_sel = 0;

static lv_obj_t *brightness_overlay = NULL;
static lv_obj_t *brightness_pct_lbl = NULL;
static lv_obj_t *language_overlay = NULL;
static lv_obj_t *language_arrows[NUM_LANG_OPTS] = { NULL };
static lv_obj_t *language_opt_lbls[NUM_LANG_OPTS] = { NULL };
static uint8_t menu_lang_sel = 0;

static menu_layer_t menu_layer = MENU_LAYER_NONE;
static Preferences prefs_backlight;
static uint8_t backlight_pct = 100;
static uint8_t ui_lang = LANG_EN;
static uint8_t shadow_theme = 0;
static lv_obj_t *main_tabs[NUM_TABS] = { NULL };

/** Latched power output mode (only one). -1 = none chosen yet. */
static int8_t pwr_out_active_idx = -1;
static bool extra_feat_on[NUM_EXTRA_OPTS] = { false, false, false };
static lv_obj_t *extra_row_mark_lbls[NUM_EXTRA_OPTS] = { NULL };
static lv_obj_t *menu_toast_overlay = NULL;
static lv_obj_t *menu_toast_lbl = NULL;
static lv_timer_t *menu_toast_timer = NULL;

static lv_obj_t *tab_title_main_lbl[NUM_TABS] = { NULL };
static lv_obj_t *tab_title_back_lbl[NUM_TABS] = { NULL };
static lv_obj_t *trip_name_lbls[2] = { NULL };
static lv_obj_t *accel_name_lbls[4] = { NULL };
static lv_obj_t *accel_val_lbls[4] = { NULL };
static lv_obj_t *fuel_inst_name_lbl = NULL;
static lv_obj_t *fuel_avg_name_lbl = NULL;
static lv_obj_t *boost_unit_lbl = NULL;
static lv_obj_t *pwr_name_lbls[4] = { NULL };
static lv_obj_t *menu_root_title_main = NULL;
static lv_obj_t *menu_root_title_back = NULL;
static lv_obj_t *pwr_title_main = NULL;
static lv_obj_t *pwr_title_back = NULL;
static lv_obj_t *extra_title_main = NULL;
static lv_obj_t *extra_title_back = NULL;
static lv_obj_t *brightness_title_main = NULL;
static lv_obj_t *brightness_title_back = NULL;
static lv_obj_t *language_title_main = NULL;
static lv_obj_t *language_title_back = NULL;

static lv_obj_t *trip_dist_lbl = NULL;
static lv_obj_t *trip_time_lbl = NULL;
static lv_obj_t *trip_avg_lbl = NULL;
static lv_obj_t *fuel_val_lbl = NULL;
static lv_obj_t *fuel_avg_lbl = NULL;
static lv_obj_t *power_val_lbl[5] = { NULL };
static lv_obj_t *aux_grid_overlay = NULL;
static lv_obj_t *boost_gauge = NULL;
static lv_obj_t *boost_val_left = NULL;
static lv_obj_t *boost_val_dot = NULL;
static lv_obj_t *boost_val_right = NULL;
static lv_obj_t *rpm_gauge = NULL;
static lv_obj_t *rpm_val_lbl = NULL;
static lv_obj_t *inj_gauge = NULL;
static lv_obj_t *inj_val_lbl = NULL;
static lv_obj_t *afr_gauge = NULL;
static lv_obj_t *afr_val_lbl = NULL;
static lv_obj_t *map_gauge = NULL;
static lv_obj_t *map_val_lbl = NULL;
static lv_obj_t *clt_gauge = NULL;
static lv_obj_t *clt_val_lbl = NULL;
static lv_obj_t *kmh_gauge = NULL;
static lv_obj_t *kmh_val_lbl = NULL;
static lv_obj_t *gps_row_name_lbl[4] = { NULL };
static lv_obj_t *gps_link_val_lbl = NULL;
static lv_obj_t *gps_fix_val_lbl = NULL;
static lv_obj_t *gps_sats_val_lbl = NULL;
static lv_obj_t *gps_age_val_lbl = NULL;
/** Arc sweep maps 0 .. RPM_GAUGE_MAX to the indicator (same half-circle as boost). */
#define RPM_GAUGE_MAX     8000.0f
#define INJ_GAUGE_MAX_MS  25.0f
#define AFR_GAUGE_MIN     10.0f
#define AFR_GAUGE_MAX     20.0f
#define MAP_GAUGE_MAX_KPA 250.0f
#define CLT_GAUGE_MAX_C   130.0f
#define KMH_GAUGE_MAX     300.0f
#define TAB_IDX_FUEL      0
#define TAB_IDX_TRIP      1
#define TAB_IDX_ACCEL     2
#define TAB_IDX_POWER     3
#define TAB_IDX_RPM       4
#define TAB_IDX_INJ       5
#define TAB_IDX_AFR       6
#define TAB_IDX_MAP       7
#define TAB_IDX_CLT       8
#define TAB_IDX_KMH       9
#define TAB_IDX_GPS       10

enum { AUX_SLOT_AFR = 0, AUX_SLOT_MAP = 1, AUX_SLOT_IAT = 2, AUX_SLOT_CLT = 3, AUX_SLOT_COUNT = 4 };
enum { AUX_METRIC_AFR = 0, AUX_METRIC_MAP, AUX_METRIC_IAT, AUX_METRIC_CLT, AUX_METRIC_PW };
static char aux_text_afr[16] = "--";
static char aux_text_map[16] = "--";
static char aux_text_iat[16] = "--";
static char aux_text_clt[16] = "--";
static char aux_text_pw[16]  = "--";

static lv_obj_t *bottom_panels[3]   = { NULL };
static lv_obj_t *bottom_icons[3]    = { NULL };
static lv_obj_t *bottom_captions[3] = { NULL };
static bool check_warning = true;
/** Debounced: assume OK until a fault is confirmed for `DASH_SENSOR_FAULT_CONFIRM_MS`. */
static bool oil_ok = true;
static bool cruise_on = false;
static bool rpm_detected = false;
static float latest_boost = 0.0f;
static float latest_rpm = 0.0f;
static float latest_inj = 0.0f;
static float latest_afr = 14.7f;
static float latest_map = 100.0f;
static float latest_clt = 80.0f;
static float latest_kmh = 0.0f;
/** Fuel card: raw `F` from Mega — >=0 instant L/100 km (moving); <0 means instant L/h is `-F` (standstill). Average from `L`. */
static float latest_F_instant = 0.0f;
static bool latest_have_F_instant = false;
static float latest_avg_l100 = 0.0f;
static bool latest_have_avg_l100 = false;
static bool gps_alive = false;
static bool gps_fix = false;
static uint8_t gps_sats = 0;
static float gps_age_s = 999.0f;

/** Active dashboard page 0..NUM_TABS-1 (N/P cycle; kept in sync with visible page). */
static uint8_t dash_active_tab = 0;
/** Skip redundant RPM label/arc LVGL calls when value unchanged (only used when RPM tab is active). */
static int32_t s_rpm_ui_rounded = -1;
static int16_t s_rpm_ui_arc_end = -1;

static lv_obj_t *oil_warning_overlay = NULL;
static lv_obj_t *oil_warning_label = NULL;
static lv_obj_t *fluid_warn_container = NULL;
static lv_obj_t *fluid_coolant_lbl = NULL;
static lv_obj_t *fluid_brake_lbl = NULL;
static bool coolant_level_ok = true;
static bool brake_fluid_ok = true;
static bool fluid_sensors_inited = false;
/** Full-screen oil alert: ST7789 + TFT_BGR reads 0xFF0000 as wrong hue; 0x0000FF reads red on this panel. */
#define COLOR_OIL_ALERT_BG  lv_color_hex(0x0000FF)
#define COLOR_BAR_NORMAL    lv_color_hex(0x404040)
// Tuned for current panel color channel behavior so on-screen colors match intent.
#define COLOR_CHECK_ACTIVE  lv_color_hex(0x00A0C4)   // displays as yellow
#define COLOR_OIL_WARNING   lv_color_hex(0x2222B2)   // displays as red
#define COLOR_CRUISE_ON     lv_color_hex(0x228B22)
/** Full-screen menu toasts — power uses same “yellow-on-panel” as check lamp */
#define COLOR_TOAST_PWR_ON    COLOR_CHECK_ACTIVE
#define COLOR_TOAST_EXTRA_ON  COLOR_CRUISE_ON
#define COLOR_TOAST_OFF       lv_color_hex(0xB0B0B0)
#define COLOR_FLUID_WARN_TEXT  COLOR_CHECK_ACTIVE

#define MEGA_LINE_MAX  48
static char mega_line_buf[MEGA_LINE_MAX];
static uint8_t mega_line_len = 0;

// LVGL flush: uses TFT_eSPI pushPixels (ESP32 SPI FIFO + optional byte swap in library).
// Not using pushPixelsDMA: DMA path byte-swaps the buffer in place, which would corrupt LVGL's reused draw buffers.
static void my_disp_flush(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map) {
  (void)disp;
  const uint32_t w = (uint32_t)(area->x2 - area->x1 + 1);
  const uint32_t h = (uint32_t)(area->y2 - area->y1 + 1);

  tft.startWrite();
  tft.setAddrWindow(area->x1, area->y1, w, h);
  tft.pushPixels((uint16_t *)px_map, w * h);
  tft.endWrite();

  lv_display_flush_ready(disp);
}

static uint32_t splash_start_ms = 0;

static void refresh_fuel_consumption_card(void)
{
  char buf[28];
  if (fuel_val_lbl) {
    if (!latest_have_F_instant) {
      lv_label_set_text_static(fuel_val_lbl, "--");
      lv_obj_set_style_text_color(fuel_val_lbl, COLOR_TEXT, 0);
    } else if (latest_F_instant >= 0.0f) {
      snprintf(buf, sizeof(buf), "%.1f L/100km", (double)latest_F_instant);
      lv_label_set_text(fuel_val_lbl, buf);
      lv_obj_set_style_text_color(fuel_val_lbl, COLOR_TEXT, 0);
    } else {
      snprintf(buf, sizeof(buf), "%.1f L/h", (double)(-latest_F_instant));
      lv_label_set_text(fuel_val_lbl, buf);
      lv_obj_set_style_text_color(fuel_val_lbl, COLOR_TEXT, 0);
    }
  }
  if (fuel_avg_lbl) {
    if (latest_have_avg_l100) {
      snprintf(buf, sizeof(buf), "%.1f L/100km", (double)latest_avg_l100);
      lv_label_set_text(fuel_avg_lbl, buf);
    } else {
      lv_label_set_text_static(fuel_avg_lbl, "--");
    }
  }
}

static void reset_trip(void) {
  if (trip_dist_lbl) lv_label_set_text_static(trip_dist_lbl, "0.0 km");
  if (trip_time_lbl) lv_label_set_text_static(trip_time_lbl, "0:00");
  if (trip_avg_lbl)  lv_label_set_text_static(trip_avg_lbl, "-- L/100km");
  latest_have_avg_l100 = false;
  latest_have_F_instant = false;
  latest_F_instant = 0.0f;
  refresh_fuel_consumption_card();
}

static lv_obj_t *shadow_label_list[32];
static uint8_t shadow_label_count = 0;

static void apply_shadow_theme(uint8_t theme) {
  shadow_theme = theme;
  lv_color_t color;
  if (theme == 0)      color = lv_color_hex(0x1A3A1A);
  else if (theme == 1) color = lv_color_hex(0x141C3C);
  else                 color = lv_color_hex(0x5C1414);

  for (int i = 0; i < NUM_TABS; i++) {
    if (main_tabs[i]) {
      lv_obj_set_style_bg_color(main_tabs[i], COLOR_BG, 0);
      lv_obj_set_style_bg_opa(main_tabs[i], LV_OPA_COVER, 0);
    }
  }

  const int32_t radius = 3;
  const int32_t off = 1;
  for (uint8_t i = 0; i < shadow_label_count; i++) {
    if (shadow_label_list[i]) {
      lv_obj_set_style_drop_shadow_color(shadow_label_list[i], color, 0);
      lv_obj_set_style_drop_shadow_radius(shadow_label_list[i], radius, 0);
      lv_obj_set_style_drop_shadow_offset_x(shadow_label_list[i], off, 0);
      lv_obj_set_style_drop_shadow_offset_y(shadow_label_list[i], off, 0);
      lv_obj_set_style_drop_shadow_opa(shadow_label_list[i], LV_OPA_70, 0);
    }
  }
}

static void add_shadow_label(lv_obj_t *lbl) {
  if (lbl && shadow_label_count < (uint8_t)(sizeof(shadow_label_list) / sizeof(shadow_label_list[0])))
    shadow_label_list[shadow_label_count++] = lbl;
}

static void apply_title_shadow(lv_obj_t *lbl) {
  if (!lbl) return;
  lv_obj_set_style_drop_shadow_color(lbl, lv_color_hex(0x050505), 0);
  lv_obj_set_style_drop_shadow_opa(lbl, LV_OPA_80, 0);
  lv_obj_set_style_drop_shadow_radius(lbl, 6, 0);
  lv_obj_set_style_drop_shadow_offset_x(lbl, 3, 0);
  lv_obj_set_style_drop_shadow_offset_y(lbl, 2, 0);
}

// Enlarge labels while keeping default LVGL font enabled.

static void update_bottom_bar(void);
static void start_oil_warning_anim(void);
static void set_boost_value(float val);
static void set_rpm_value(float val);
static void set_injector_pw_value(float val);
static void set_afr_gauge_value(float val);
static void set_map_gauge_value(float val);
static void set_clt_gauge_value(float val);
static void set_kmh_value(float val);
static void refresh_active_gauge_page(void);
static void update_gps_monitor_widgets(void);
static void update_pwr_out_arrows(void);
static void update_language_arrows(void);
static void update_aux_sensor_grids(void);
static void poll_oil_pressure_switch(void);
static void poll_fluid_level_switches(void);
static void update_fluid_warning_strip(void);
static void buzzer_set(bool on);
static void buzzer_tick(uint32_t now_ms);
static void dash_show_tab(uint8_t idx);

static const char *const k_root_menu_en[NUM_MENU_ROOT_OPTS] = {
  "Power output", "Extra features", "Brightness", "Language"
};
static const char *const k_root_menu_lv[NUM_MENU_ROOT_OPTS] = {
  "Jaudas rezims", "Papildu funkcijas", "Spilgtums", "Valoda"
};
static const char *const k_pwr_mode_en[NUM_PWR_OUT_OPTS] = { "Low", "Medium", "All Hail Quattro" };
static const char *const k_pwr_mode_lv[NUM_PWR_OUT_OPTS] = { "Zems", "Videjs", "Visu slavu Quattro" };
static const char *const k_extra_feat_en[NUM_EXTRA_OPTS] = { "Rotational idle", "FlatShift", "Rolling launch" };
static const char *const k_extra_feat_lv[NUM_EXTRA_OPTS] = { "Rotacijas tuksgaita", "FlatShift", "Ritosais starts" };
static const char *const k_lang_opts_en[NUM_LANG_OPTS] = { "English", "Latvian" };
static const char *const k_lang_opts_lv[NUM_LANG_OPTS] = { "Anglu", "Latviesu" };

static inline const char *tr2(const char *en, const char *lv) {
  return (ui_lang == LANG_LV) ? lv : en;
}
static inline const char *pwr_mode_lbl(uint8_t i) {
  return (ui_lang == LANG_LV) ? k_pwr_mode_lv[i] : k_pwr_mode_en[i];
}
static inline const char *extra_feat_lbl(uint8_t i) {
  return (ui_lang == LANG_LV) ? k_extra_feat_lv[i] : k_extra_feat_en[i];
}
static inline void set_label_text_i18n(lv_obj_t *lbl, const char *txt) {
  if (lbl) lv_label_set_text_static(lbl, txt);
}
static inline void set_label_text_i18n_dyn(lv_obj_t *lbl, const char *txt) {
  if (lbl) lv_label_set_text(lbl, txt);
}

static uint8_t aux_metric_for(uint8_t tab_idx, uint8_t slot) {
  if (tab_idx == TAB_IDX_AFR && slot == AUX_SLOT_AFR) return AUX_METRIC_PW; // AFR page
  if (tab_idx == TAB_IDX_MAP && slot == AUX_SLOT_MAP) return AUX_METRIC_PW; // MAP page
  if (tab_idx == TAB_IDX_CLT && slot == AUX_SLOT_CLT) return AUX_METRIC_PW; // CLT page
  return slot;
}

static const char *aux_metric_name(uint8_t metric) {
  switch (metric) {
    case AUX_METRIC_AFR: return "AFR";
    case AUX_METRIC_MAP: return "MAP";
    case AUX_METRIC_IAT: return "IAT";
    case AUX_METRIC_CLT: return "CLT";
    case AUX_METRIC_PW:  return "PW";
    default:             return "--";
  }
}

static const char *aux_metric_value_text(uint8_t metric) {
  switch (metric) {
    case AUX_METRIC_AFR: return aux_text_afr;
    case AUX_METRIC_MAP: return aux_text_map;
    case AUX_METRIC_IAT: return aux_text_iat;
    case AUX_METRIC_CLT: return aux_text_clt;
    case AUX_METRIC_PW:  return aux_text_pw;
    default:             return "--";
  }
}

static void update_aux_sensor_grids(void) {
  for (uint8_t slot = 0; slot < AUX_SLOT_COUNT; slot++) {
    const uint8_t metric = aux_metric_for(dash_active_tab, slot);
    if (pwr_name_lbls[slot])
      lv_label_set_text_static(pwr_name_lbls[slot], aux_metric_name(metric));
    /* Values live in shared aux_text_* buffers that parse_mega_line updates — never two set_text_static to the same buffer. */
    if (power_val_lbl[slot + 1])
      lv_label_set_text(power_val_lbl[slot + 1], aux_metric_value_text(metric));
  }
}

static void update_gps_monitor_widgets(void) {
  if (gps_link_val_lbl) {
    lv_label_set_text_static(gps_link_val_lbl, gps_alive ? "ONLINE" : "OFFLINE");
    lv_obj_set_style_text_color(gps_link_val_lbl, gps_alive ? COLOR_CRUISE_ON : COLOR_TOAST_OFF, 0);
  }
  if (gps_fix_val_lbl) {
    lv_label_set_text_static(gps_fix_val_lbl, gps_fix ? "FIX" : "NO FIX");
    lv_obj_set_style_text_color(gps_fix_val_lbl, gps_fix ? COLOR_CRUISE_ON : COLOR_TOAST_OFF, 0);
  }
  if (gps_sats_val_lbl) {
    char b[12];
    snprintf(b, sizeof(b), "%u", (unsigned)gps_sats);
    lv_label_set_text(gps_sats_val_lbl, b);
  }
  if (gps_age_val_lbl) {
    if (gps_age_s > 99.0f)
      lv_label_set_text_static(gps_age_val_lbl, ">99s");
    else {
      char b[12];
      snprintf(b, sizeof(b), "%.1fs", (double)gps_age_s);
      lv_label_set_text(gps_age_val_lbl, b);
    }
  }
}

static void ui_lang_save_to_nvs(void) {
  prefs_backlight.begin("dash", false);
  prefs_backlight.putUChar("lang", ui_lang);
  prefs_backlight.end();
}

/** Same rationale as backlight: NVS put can block; never do it inside UART/LVGL command handling. */
static uint32_t ui_lang_nvs_save_after_ms = 0;

static void ui_lang_schedule_nvs_save(void) {
  ui_lang_nvs_save_after_ms = millis() + 2u;
}

static void ui_lang_flush_nvs_if_due(uint32_t now_ms) {
  if (ui_lang_nvs_save_after_ms == 0u)
    return;
  if ((int32_t)(now_ms - ui_lang_nvs_save_after_ms) < 0)
    return;
  ui_lang_nvs_save_after_ms = 0u;
  ui_lang_save_to_nvs();
}

static void ui_lang_load_from_nvs(void) {
  prefs_backlight.begin("dash", true);
  const uint8_t v = prefs_backlight.getUChar("lang", (uint8_t)LANG_EN);
  prefs_backlight.end();
  ui_lang = (v == (uint8_t)LANG_LV) ? (uint8_t)LANG_LV : (uint8_t)LANG_EN;
}

static void apply_ui_language(void) {
  if (tab_title_main_lbl[0]) {
    set_label_text_i18n(tab_title_main_lbl[0], tr2("Fuel consumption", "Degvielas paterins"));
    set_label_text_i18n(tab_title_back_lbl[0], tr2("Fuel consumption", "Degvielas paterins"));
    set_label_text_i18n(tab_title_main_lbl[1], tr2("Trip computer", "Brauciena dators"));
    set_label_text_i18n(tab_title_back_lbl[1], tr2("Trip computer", "Brauciena dators"));
    set_label_text_i18n(tab_title_main_lbl[2], tr2("Acceleration", "Paatrinajums"));
    set_label_text_i18n(tab_title_back_lbl[2], tr2("Acceleration", "Paatrinajums"));
    set_label_text_i18n(tab_title_main_lbl[3], tr2("Power", "Jauda"));
    set_label_text_i18n(tab_title_back_lbl[3], tr2("Power", "Jauda"));
    set_label_text_i18n(tab_title_main_lbl[4], "RPM");
    set_label_text_i18n(tab_title_back_lbl[4], "RPM");
    set_label_text_i18n(tab_title_main_lbl[5], tr2("Injector PW", "Injektora PW"));
    set_label_text_i18n(tab_title_back_lbl[5], tr2("Injector PW", "Injektora PW"));
    set_label_text_i18n(tab_title_main_lbl[6], "AFR");
    set_label_text_i18n(tab_title_back_lbl[6], "AFR");
    set_label_text_i18n(tab_title_main_lbl[7], "MAP");
    set_label_text_i18n(tab_title_back_lbl[7], "MAP");
    set_label_text_i18n(tab_title_main_lbl[8], "CLT");
    set_label_text_i18n(tab_title_back_lbl[8], "CLT");
    set_label_text_i18n(tab_title_main_lbl[9], "KMH");
    set_label_text_i18n(tab_title_back_lbl[9], "KMH");
    set_label_text_i18n(tab_title_main_lbl[10], tr2("GPS monitor", "GPS monitors"));
    set_label_text_i18n(tab_title_back_lbl[10], tr2("GPS monitor", "GPS monitors"));
  }
  if (fuel_inst_name_lbl) set_label_text_i18n(fuel_inst_name_lbl, tr2("Instant", "Momentanais"));
  if (fuel_avg_name_lbl)  set_label_text_i18n(fuel_avg_name_lbl,  tr2("Average", "Videjais"));
  if (trip_name_lbls[0])  set_label_text_i18n(trip_name_lbls[0],  tr2("Distance", "Attalums"));
  if (trip_name_lbls[1])  set_label_text_i18n(trip_name_lbls[1],  tr2("Time", "Laiks"));
  {
    static const char *const an_en[4] = { "0-60 km/h", "0-100 km/h", "0-120 km/h", "60-120 km/h" };
    static const char *const an_lv[4] = { "0-60 km/h", "0-100 km/h", "0-120 km/h", "60-120 km/h" };
    for (int i = 0; i < 4; i++) {
      if (accel_name_lbls[i])
        set_label_text_i18n(accel_name_lbls[i], tr2(an_en[i], an_lv[i]));
    }
  }
  if (boost_unit_lbl)     set_label_text_i18n(boost_unit_lbl,     tr2("Bar", "Bar"));
  if (pwr_name_lbls[0])   set_label_text_i18n(pwr_name_lbls[0],   "AFR");
  if (pwr_name_lbls[1])   set_label_text_i18n(pwr_name_lbls[1],   "MAP");
  if (pwr_name_lbls[2])   set_label_text_i18n(pwr_name_lbls[2],   "IAT");
  if (pwr_name_lbls[3])   set_label_text_i18n(pwr_name_lbls[3],   "CLT");
  if (gps_row_name_lbl[0]) set_label_text_i18n(gps_row_name_lbl[0], tr2("Link", "Savienojums"));
  if (gps_row_name_lbl[1]) set_label_text_i18n(gps_row_name_lbl[1], tr2("Fix", "Fikss"));
  if (gps_row_name_lbl[2]) set_label_text_i18n(gps_row_name_lbl[2], tr2("Satellites", "Sateliti"));
  if (gps_row_name_lbl[3]) set_label_text_i18n(gps_row_name_lbl[3], tr2("Age", "Vecums"));

  if (menu_root_title_main) set_label_text_i18n(menu_root_title_main, tr2("Menu", "Izvelne"));
  if (menu_root_title_back) set_label_text_i18n(menu_root_title_back, tr2("Menu", "Izvelne"));
  if (pwr_title_main)       set_label_text_i18n(pwr_title_main,       tr2("Power output", "Jaudas rezims"));
  if (pwr_title_back)       set_label_text_i18n(pwr_title_back,       tr2("Power output", "Jaudas rezims"));
  if (extra_title_main)     set_label_text_i18n(extra_title_main,     tr2("Extra features", "Papildu funkcijas"));
  if (extra_title_back)     set_label_text_i18n(extra_title_back,     tr2("Extra features", "Papildu funkcijas"));
  if (brightness_title_main) set_label_text_i18n(brightness_title_main, tr2("Brightness", "Spilgtums"));
  if (brightness_title_back) set_label_text_i18n(brightness_title_back, tr2("Brightness", "Spilgtums"));
  if (language_title_main)   set_label_text_i18n(language_title_main,   tr2("Language", "Valoda"));
  if (language_title_back)   set_label_text_i18n(language_title_back,   tr2("Language", "Valoda"));

  for (int i = 0; i < NUM_MENU_ROOT_OPTS; i++) set_label_text_i18n(menu_root_opt_lbls[i], (ui_lang == LANG_LV) ? k_root_menu_lv[i] : k_root_menu_en[i]);
  for (int i = 0; i < NUM_PWR_OUT_OPTS; i++) set_label_text_i18n(pwr_out_opt_lbls[i], pwr_mode_lbl((uint8_t)i));
  // These rows use LV_LABEL_LONG_MODE_DOTS, so keep dynamic text buffers for LVGL's internal edits.
  for (int i = 0; i < NUM_EXTRA_OPTS; i++) set_label_text_i18n_dyn(extra_opt_lbls[i], extra_feat_lbl((uint8_t)i));
  for (int i = 0; i < NUM_LANG_OPTS; i++) set_label_text_i18n(language_opt_lbls[i], (ui_lang == LANG_LV) ? k_lang_opts_lv[i] : k_lang_opts_en[i]);

  set_label_text_i18n(bottom_captions[0], tr2("Check", "Parbaude"));
  set_label_text_i18n(bottom_captions[1], tr2("Oil", "Ella"));
  set_label_text_i18n(bottom_captions[2], tr2("Cruise", "Kruizs"));
  // Scroll-circular labels can manipulate text presentation; use dynamic text here.
  set_label_text_i18n_dyn(fluid_coolant_lbl, tr2("Coolant level too low!", "Par zemu dzesesanas skidrums!"));
  set_label_text_i18n_dyn(fluid_brake_lbl, tr2("Brake fluid too low!", "Par zemu bremzu skidums!"));
}

static void update_oil_warning_overlay(void) {
  static bool oil_anim_armed = false;
  if (!oil_warning_overlay) return;
  const bool active = rpm_detected && !oil_ok;
  if (active) {
    lv_obj_clear_flag(oil_warning_overlay, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(oil_warning_overlay);
    if (!oil_anim_armed) {
      start_oil_warning_anim();
      oil_anim_armed = true;
    }
  } else {
    oil_anim_armed = false;
    lv_obj_add_flag(oil_warning_overlay, LV_OBJ_FLAG_HIDDEN);
    if (oil_warning_label) {
      lv_anim_del(oil_warning_label, NULL);
      lv_obj_set_style_opa(oil_warning_label, LV_OPA_COVER, 0);
    }
  }
}

static void oil_warning_anim_cb(void *obj, int32_t v) {
  lv_obj_set_style_opa((lv_obj_t *)obj, (lv_opa_t)v, 0);
}

static void start_oil_warning_anim(void) {
  if (!oil_warning_label) return;
  lv_anim_del(oil_warning_label, NULL);

  lv_anim_t a;
  lv_anim_init(&a);
  lv_anim_set_var(&a, oil_warning_label);
  lv_anim_set_values(&a, 0, 255);
  lv_anim_set_time(&a, 1000);
  lv_anim_set_playback_time(&a, 1000);
  lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);
  lv_anim_set_exec_cb(&a, oil_warning_anim_cb);
  lv_anim_start(&a);
}

/** Map 0..1 to arc indicator 180..359 (never 360) — avoids fragile paths with half-circle arcs + flush. */
static int16_t arc_norm_to_end_angle(float norm) {
  if (norm < 0.0f) norm = 0.0f;
  if (norm > 1.0f) norm = 1.0f;
  int32_t end = 180 + (int32_t)(norm * 179.0f + 0.5f);
  if (end > 359) end = 359;
  if (end < 180) end = 180;
  return (int16_t)end;
}

static void apply_boost_gauge_arc(float val) {
  if (!boost_gauge) return;
  float v = val;
  if (v < -1.0f) v = -1.0f;
  if (v > 2.0f) v = 2.0f;
  float norm = (v + 1.0f) / 3.0f;
  lv_arc_set_angles(boost_gauge, 180, arc_norm_to_end_angle(norm));
}

// Parse one live‑data line from Mega (same logic as original)
static void parse_mega_line(void) {
  if (mega_line_len == 0) return;
  mega_line_buf[mega_line_len] = '\0';
  char tag = mega_line_buf[0];
  const char *payload = (mega_line_len > 1) ? &mega_line_buf[1] : "";
  float val = 0.0f;
  if (mega_line_len > 1) val = (float)atof(payload);
  bool aux_dirty = false;

  switch (tag) {
    case 'F':
      latest_have_F_instant = true;
      latest_F_instant = val;
      refresh_fuel_consumption_card();
      break;
    case 'D':
      if (trip_dist_lbl) {
        char b[20];
        snprintf(b, sizeof(b), "%.1f km", (double)val);
        lv_label_set_text(trip_dist_lbl, b);
      }
      break;
    case 'T':
      if (trip_time_lbl) {
        char b[12];
        if (strchr(payload, ':') != NULL) {
          int mins = 0, sec = 0;
          sscanf(payload, "%d:%d", &mins, &sec);
          snprintf(b, sizeof(b), "%d:%02d", mins, sec);
        } else {
          int total_mins = (int)val;
          snprintf(b, sizeof(b), "%d:%02d", total_mins / 60, total_mins % 60);
        }
        lv_label_set_text(trip_time_lbl, b);
      }
      break;
    case 'L':
      if (val < 0.0f) {
        latest_have_avg_l100 = false;
        if (trip_avg_lbl)
          lv_label_set_text_static(trip_avg_lbl, "-- L/100km");
      } else {
        latest_avg_l100 = val;
        latest_have_avg_l100 = true;
        if (trip_avg_lbl) {
          char b[24];
          snprintf(b, sizeof(b), "%.1f L/100km", (double)val);
          lv_label_set_text(trip_avg_lbl, b);
        }
      }
      refresh_fuel_consumption_card();
      break;
    case 'B':
      latest_boost = val;
      if (dash_active_tab == TAB_IDX_POWER) {
        set_boost_value(val);
        apply_boost_gauge_arc(val);
      }
      break;
    case 'E':
      latest_rpm = val;
      if (dash_active_tab == TAB_IDX_RPM)
        set_rpm_value(val);
      break;
    case 'J':
      latest_inj = val;
      if (dash_active_tab == TAB_IDX_INJ)
        set_injector_pw_value(val);
      snprintf(aux_text_pw, sizeof(aux_text_pw), "%.2f ms", (double)val);
      aux_dirty = true;
      break;
    case 'X':
      latest_afr = val;
      if (dash_active_tab == TAB_IDX_AFR)
        set_afr_gauge_value(val);
      snprintf(aux_text_afr, sizeof(aux_text_afr), "%.2f", (double)val);
      aux_dirty = true;
      break;
    case 'M':
      latest_map = val;
      if (dash_active_tab == TAB_IDX_MAP)
        set_map_gauge_value(val);
      snprintf(aux_text_map, sizeof(aux_text_map), "%.0f kPa", (double)val);
      aux_dirty = true;
      break;
    case 'I':
      snprintf(aux_text_iat, sizeof(aux_text_iat), "%.0f C", (double)val);
      aux_dirty = true;
      break;
    case 'C':
      latest_clt = val;
      if (dash_active_tab == TAB_IDX_CLT)
        set_clt_gauge_value(val);
      snprintf(aux_text_clt, sizeof(aux_text_clt), "%.0f C", (double)val);
      aux_dirty = true;
      break;
    case 'V':
      /* STM32: ECU vehicle speed (VSS / vehicleSpeedKph), not GPS SOG. */
      latest_kmh = val;
      if (dash_active_tab == TAB_IDX_KMH)
        set_kmh_value(val);
      break;
    case 'Y':
      gps_alive = (val > 0.5f);
      update_gps_monitor_widgets();
      break;
    case 'Z':
      gps_fix = (val > 0.5f);
      update_gps_monitor_widgets();
      break;
    case 'H':
      if (val < 0.0f) val = 0.0f;
      if (val > 99.0f) val = 99.0f;
      gps_sats = (uint8_t)(val + 0.5f);
      update_gps_monitor_widgets();
      break;
    case 'g':
      if (val < 0.0f) val = 0.0f;
      gps_age_s = val;
      update_gps_monitor_widgets();
      break;
    case 'a':
      if (accel_val_lbls[0]) {
        if (val < 0.0f)
          lv_label_set_text_static(accel_val_lbls[0], "--");
        else {
          char b[16];
          snprintf(b, sizeof(b), "%.2f s", (double)val);
          lv_label_set_text(accel_val_lbls[0], b);
        }
      }
      break;
    case 'b':
      if (accel_val_lbls[1]) {
        if (val < 0.0f)
          lv_label_set_text_static(accel_val_lbls[1], "--");
        else {
          char b[16];
          snprintf(b, sizeof(b), "%.2f s", (double)val);
          lv_label_set_text(accel_val_lbls[1], b);
        }
      }
      break;
    case 'c':
      if (accel_val_lbls[2]) {
        if (val < 0.0f)
          lv_label_set_text_static(accel_val_lbls[2], "--");
        else {
          char b[16];
          snprintf(b, sizeof(b), "%.2f s", (double)val);
          lv_label_set_text(accel_val_lbls[2], b);
        }
      }
      break;
    case 'd':
      if (accel_val_lbls[3]) {
        if (val < 0.0f)
          lv_label_set_text_static(accel_val_lbls[3], "--");
        else {
          char b[16];
          snprintf(b, sizeof(b), "%.2f s", (double)val);
          lv_label_set_text(accel_val_lbls[3], b);
        }
      }
      break;
    case 'K':
      check_warning = (val != 0.0f);
      update_bottom_bar();
      break;
    case 'W':
      // Oil is read on OIL_PRESSURE_SWITCH_PIN; ignore legacy gateway W0/W1.
      break;
    case 'R': {
      const bool nd = (val != 0.0f);
      if (nd != rpm_detected) {
        rpm_detected = nd;
        update_oil_warning_overlay();
      }
      break;
    }
    case 'U':
      cruise_on = (val != 0.0f);
      update_bottom_bar();
      break;
    default:
      break;
  }
  if (aux_dirty)
    update_aux_sensor_grids();
  mega_line_len = 0;
}

static void poll_oil_pressure_switch(void) {
  const uint32_t now_ms = millis();
  const bool raw_ok = (digitalRead(OIL_PRESSURE_SWITCH_PIN) == LOW);
  static uint32_t oil_fault_since_ms = 0u;
  const bool prev = oil_ok;

  if (raw_ok) {
    oil_fault_since_ms = 0u;
    oil_ok = true;
  } else {
    if (oil_fault_since_ms == 0u)
      oil_fault_since_ms = now_ms;
    else if (oil_ok && (now_ms - oil_fault_since_ms >= DASH_SENSOR_FAULT_CONFIRM_MS))
      oil_ok = false;
  }

  if (prev != oil_ok) {
    update_bottom_bar();
    update_oil_warning_overlay();
  }
}

static void update_fluid_warning_strip(void) {
  if (!fluid_warn_container) return;
#if !DASH_FLUID_LEVEL_WARNINGS
  lv_obj_add_flag(fluid_warn_container, LV_OBJ_FLAG_HIDDEN);
  return;
#endif
  const bool coolant_warn = !coolant_level_ok;
  const bool brake_warn = !brake_fluid_ok;
  if (!coolant_warn && !brake_warn) {
    lv_obj_add_flag(fluid_warn_container, LV_OBJ_FLAG_HIDDEN);
    return;
  }
  lv_obj_clear_flag(fluid_warn_container, LV_OBJ_FLAG_HIDDEN);
  if (fluid_coolant_lbl) {
    if (coolant_warn)
      lv_obj_clear_flag(fluid_coolant_lbl, LV_OBJ_FLAG_HIDDEN);
    else
      lv_obj_add_flag(fluid_coolant_lbl, LV_OBJ_FLAG_HIDDEN);
  }
  if (fluid_brake_lbl) {
    if (brake_warn)
      lv_obj_clear_flag(fluid_brake_lbl, LV_OBJ_FLAG_HIDDEN);
    else
      lv_obj_add_flag(fluid_brake_lbl, LV_OBJ_FLAG_HIDDEN);
  }
  lv_obj_move_foreground(fluid_warn_container);
}

static void poll_fluid_level_switches(void) {
#if !DASH_FLUID_LEVEL_WARNINGS
  return;
#else
  const uint32_t now_ms = millis();
  const bool raw_c_ok = (digitalRead(COOLANT_LEVEL_PIN) == HIGH);
  const bool raw_b_ok = (digitalRead(BRAKE_FLUID_PIN) == HIGH);
  static uint32_t coolant_fault_since_ms = 0u;
  static uint32_t brake_fault_since_ms = 0u;
  const bool prev_c = coolant_level_ok;
  const bool prev_b = brake_fluid_ok;

  if (raw_c_ok) {
    coolant_fault_since_ms = 0u;
    coolant_level_ok = true;
  } else {
    if (coolant_fault_since_ms == 0u)
      coolant_fault_since_ms = now_ms;
    else if (coolant_level_ok && (now_ms - coolant_fault_since_ms >= DASH_SENSOR_FAULT_CONFIRM_MS))
      coolant_level_ok = false;
  }

  if (raw_b_ok) {
    brake_fault_since_ms = 0u;
    brake_fluid_ok = true;
  } else {
    if (brake_fault_since_ms == 0u)
      brake_fault_since_ms = now_ms;
    else if (brake_fluid_ok && (now_ms - brake_fault_since_ms >= DASH_SENSOR_FAULT_CONFIRM_MS))
      brake_fluid_ok = false;
  }

  if (fluid_sensors_inited && prev_c == coolant_level_ok && prev_b == brake_fluid_ok)
    return;
  fluid_sensors_inited = true;
  update_fluid_warning_strip();
#endif
}

static void buzzer_set(bool on) {
#if BUZZER_ACTIVE_HIGH
  digitalWrite(BUZZER_PIN, on ? HIGH : LOW);
#else
  digitalWrite(BUZZER_PIN, on ? LOW : HIGH);
#endif
}

static void buzzer_tick(uint32_t now_ms) {
  typedef enum {
    BZ_IDLE = 0,
    BZ_OIL_ON,
    BZ_OIL_OFF,
    BZ_W_B1_ON,
    BZ_W_GAP1,
    BZ_W_B2_ON,
    BZ_W_DONE,
  } buzz_phase_t;

  static buzz_phase_t phase = BZ_IDLE;
  static uint32_t t_phase = 0;
  static bool prev_coolant_bad = false;
  static bool prev_brake_bad = false;
  static bool prev_engine_bad = false;

  const bool oil_alarm = rpm_detected && !oil_ok;
  const bool coolant_bad = !coolant_level_ok;
  const bool brake_bad = !brake_fluid_ok;
  const bool engine_bad = check_warning;
  const bool warn_alarm = coolant_bad || brake_bad || engine_bad;

  if (!oil_alarm && !warn_alarm) {
    buzzer_set(false);
    phase = BZ_IDLE;
    prev_coolant_bad = false;
    prev_brake_bad = false;
    prev_engine_bad = false;
    return;
  }

  if (oil_alarm) {
    if (phase != BZ_OIL_ON && phase != BZ_OIL_OFF) {
      phase = BZ_OIL_ON;
      t_phase = now_ms;
    }
    const uint32_t dt = now_ms - t_phase;
    if (phase == BZ_OIL_ON) {
      buzzer_set(true);
      if (dt >= 1000u) {
        phase = BZ_OIL_OFF;
        t_phase = now_ms;
      }
    } else {
      buzzer_set(false);
      if (dt >= 1000u) {
        phase = BZ_OIL_ON;
        t_phase = now_ms;
      }
    }
    prev_coolant_bad = coolant_bad;
    prev_brake_bad = brake_bad;
    prev_engine_bad = engine_bad;
    return;
  }

  const bool edge_coolant = coolant_bad && !prev_coolant_bad;
  const bool edge_brake = brake_bad && !prev_brake_bad;
  const bool edge_engine = engine_bad && !prev_engine_bad;
  const bool any_warn_edge = edge_coolant || edge_brake || edge_engine;

  prev_coolant_bad = coolant_bad;
  prev_brake_bad = brake_bad;
  prev_engine_bad = engine_bad;

  if (any_warn_edge) {
    phase = BZ_W_B1_ON;
    t_phase = now_ms;
  }

  const uint32_t dt = now_ms - t_phase;
  switch (phase) {
    case BZ_W_B1_ON:
      buzzer_set(true);
      if (dt >= 90u) {
        phase = BZ_W_GAP1;
        t_phase = now_ms;
      }
      break;
    case BZ_W_GAP1:
      buzzer_set(false);
      if (dt >= 80u) {
        phase = BZ_W_B2_ON;
        t_phase = now_ms;
      }
      break;
    case BZ_W_B2_ON:
      buzzer_set(true);
      if (dt >= 90u) {
        phase = BZ_W_DONE;
        buzzer_set(false);
      }
      break;
    case BZ_W_DONE:
      buzzer_set(false);
      break;
    default:
      buzzer_set(false);
      break;
  }
}

static void set_boost_value(float val) {
  if (!boost_val_left || !boost_val_dot || !boost_val_right) return;

  if (val < -9.9f) val = -9.9f;
  if (val >  9.9f) val =  9.9f;

  int sign = (val < 0.0f) ? -1 : 1;
  float abs_v = fabsf(val);
  int whole = (int)abs_v;
  int frac  = (int)roundf((abs_v - (float)whole) * 10.0f);
  if (frac >= 10) {
    frac = 0;
    whole += 1;
  }

  static char left[8];
  static char right[8];
  if (sign < 0)
    snprintf(left, sizeof(left), "-%d", whole);
  else
    snprintf(left, sizeof(left), "%d", whole);
  snprintf(right, sizeof(right), "%d", frac);

  lv_label_set_text_static(boost_val_left, left);
  lv_label_set_text_static(boost_val_right, right);
}

static void rpm_invalidate_ui_dedupe(void) {
  s_rpm_ui_rounded = -1;
  s_rpm_ui_arc_end = -1;
}

static void set_rpm_value(float val) {
  if (!rpm_val_lbl) return;
  if (val < 0.0f) val = 0.0f;
  if (val > 99999.0f) val = 99999.0f;
  const int32_t rounded = (int32_t)(val + 0.5f);
  int16_t arc_end = 180;
  if (rpm_gauge) {
    float g = val;
    if (g > RPM_GAUGE_MAX) g = RPM_GAUGE_MAX;
    float norm = g / RPM_GAUGE_MAX;
    arc_end = arc_norm_to_end_angle(norm);
  }
  if (rounded == s_rpm_ui_rounded && (!rpm_gauge || arc_end == s_rpm_ui_arc_end))
    return;
  s_rpm_ui_rounded = rounded;
  s_rpm_ui_arc_end = arc_end;

  char buf[16];
  snprintf(buf, sizeof(buf), "%.0f", (double)val);
  lv_label_set_text(rpm_val_lbl, buf);
  if (rpm_gauge)
    lv_arc_set_angles(rpm_gauge, 180, arc_end);
}

static void set_injector_pw_value(float val) {
  if (!inj_val_lbl) return;
  if (val < 0.0f) val = 0.0f;
  if (val > 99.99f) val = 99.99f;
  char buf[16];
  snprintf(buf, sizeof(buf), "%.2f", (double)val);
  lv_label_set_text(inj_val_lbl, buf);
  if (inj_gauge) {
    float g = val;
    if (g > INJ_GAUGE_MAX_MS) g = INJ_GAUGE_MAX_MS;
    float norm = g / INJ_GAUGE_MAX_MS;
    lv_arc_set_angles(inj_gauge, 180, arc_norm_to_end_angle(norm));
  }
}

static void set_afr_gauge_value(float val) {
  if (!afr_val_lbl) return;
  char buf[16];
  snprintf(buf, sizeof(buf), "%.2f", (double)val);
  lv_label_set_text(afr_val_lbl, buf);
  if (afr_gauge) {
    float v = val;
    if (v < AFR_GAUGE_MIN) v = AFR_GAUGE_MIN;
    if (v > AFR_GAUGE_MAX) v = AFR_GAUGE_MAX;
    float norm = (v - AFR_GAUGE_MIN) / (AFR_GAUGE_MAX - AFR_GAUGE_MIN);
    lv_arc_set_angles(afr_gauge, 180, arc_norm_to_end_angle(norm));
  }
}

static void set_map_gauge_value(float val) {
  if (!map_val_lbl) return;
  if (val < 0.0f) val = 0.0f;
  char buf[16];
  snprintf(buf, sizeof(buf), "%.0f", (double)val);
  lv_label_set_text(map_val_lbl, buf);
  if (map_gauge) {
    float g = val;
    if (g > MAP_GAUGE_MAX_KPA) g = MAP_GAUGE_MAX_KPA;
    float norm = g / MAP_GAUGE_MAX_KPA;
    lv_arc_set_angles(map_gauge, 180, arc_norm_to_end_angle(norm));
  }
}

static void set_clt_gauge_value(float val) {
  if (!clt_val_lbl) return;
  char buf[16];
  snprintf(buf, sizeof(buf), "%.0f", (double)val);
  lv_label_set_text(clt_val_lbl, buf);
  if (clt_gauge) {
    float g = val;
    if (g < 0.0f) g = 0.0f;
    if (g > CLT_GAUGE_MAX_C) g = CLT_GAUGE_MAX_C;
    float norm = g / CLT_GAUGE_MAX_C;
    lv_arc_set_angles(clt_gauge, 180, arc_norm_to_end_angle(norm));
  }
}

static void set_kmh_value(float val) {
  if (!kmh_val_lbl) return;
  if (val < 0.0f) val = 0.0f;
  if (val > 999.0f) val = 999.0f;
  char buf[16];
  snprintf(buf, sizeof(buf), "%.0f", (double)val);
  lv_label_set_text(kmh_val_lbl, buf);
  if (kmh_gauge) {
    float g = val;
    if (g > KMH_GAUGE_MAX) g = KMH_GAUGE_MAX;
    float norm = g / KMH_GAUGE_MAX;
    lv_arc_set_angles(kmh_gauge, 180, arc_norm_to_end_angle(norm));
  }
}

static void refresh_active_gauge_page(void) {
  switch (dash_active_tab) {
    case TAB_IDX_FUEL:
      refresh_fuel_consumption_card();
      break;
    case TAB_IDX_POWER:
      set_boost_value(latest_boost);
      apply_boost_gauge_arc(latest_boost);
      break;
    case TAB_IDX_RPM:
      set_rpm_value(latest_rpm);
      break;
    case TAB_IDX_INJ:
      set_injector_pw_value(latest_inj);
      break;
    case TAB_IDX_AFR:
      set_afr_gauge_value(latest_afr);
      break;
    case TAB_IDX_MAP:
      set_map_gauge_value(latest_map);
      break;
    case TAB_IDX_CLT:
      set_clt_gauge_value(latest_clt);
      break;
    case TAB_IDX_KMH:
      set_kmh_value(latest_kmh);
      break;
    default:
      break;
  }
}

static void dash_show_tab(uint8_t idx) {
  if (!main_tabview || idx >= NUM_TABS) return;
  for (int i = 0; i < NUM_TABS; i++) {
    if (!main_tabs[i]) continue;
    if ((uint8_t)i == idx)
      lv_obj_remove_flag(main_tabs[i], LV_OBJ_FLAG_HIDDEN);
    else
      lv_obj_add_flag(main_tabs[i], LV_OBJ_FLAG_HIDDEN);
  }
  dash_active_tab = idx;
  if (aux_grid_overlay) {
    const bool show_aux = (idx >= TAB_IDX_POWER && idx <= TAB_IDX_KMH);
    if (show_aux) lv_obj_clear_flag(aux_grid_overlay, LV_OBJ_FLAG_HIDDEN);
    else          lv_obj_add_flag(aux_grid_overlay, LV_OBJ_FLAG_HIDDEN);
  }
  update_aux_sensor_grids();
  refresh_active_gauge_page();
}

static void backlight_hw_init(void) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcAttach(TFT_BACKLIGHT_PIN, BACKLIGHT_PWM_HZ, BACKLIGHT_PWM_BITS);
#else
  const uint8_t ch = 7;
  ledcSetup(ch, BACKLIGHT_PWM_HZ, BACKLIGHT_PWM_BITS);
  ledcAttachPin(TFT_BACKLIGHT_PIN, ch);
#endif
}

static void backlight_hw_set_duty(uint8_t duty0_255) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcWrite(TFT_BACKLIGHT_PIN, duty0_255);
#else
  ledcWrite(7, duty0_255);
#endif
}

static void backlight_apply_pct(uint8_t pct) {
  if (pct < BACKLIGHT_MIN_PCT)
    pct = BACKLIGHT_MIN_PCT;
  if (pct > 100u)
    pct = 100u;
  backlight_pct = pct;
  const uint32_t maxduty = (1u << BACKLIGHT_PWM_BITS) - 1u;
  uint32_t d = (uint32_t)pct * maxduty / 100u;
  if (d > maxduty)
    d = maxduty;
  backlight_hw_set_duty((uint8_t)d);
}

static void backlight_load_from_nvs(void) {
  prefs_backlight.begin("dash", true);
  const uint8_t v = prefs_backlight.getUChar("bl", 100);
  prefs_backlight.end();
  backlight_apply_pct(v);
}

static void backlight_save_to_nvs(void) {
  prefs_backlight.begin("dash", false);
  prefs_backlight.putUChar("bl", backlight_pct);
  prefs_backlight.end();
}

/** NVS flash writes can block for milliseconds; never call from UART/LVGL command paths — schedule from loop(). */
static uint32_t backlight_nvs_save_after_ms = 0;

static void backlight_schedule_nvs_save(void) {
  backlight_nvs_save_after_ms = millis() + 800u;
}

static void backlight_flush_nvs_if_due(uint32_t now_ms) {
  if (backlight_nvs_save_after_ms == 0u)
    return;
  if ((int32_t)(now_ms - backlight_nvs_save_after_ms) < 0)
    return;
  backlight_nvs_save_after_ms = 0u;
  backlight_save_to_nvs();
}

static void sync_brightness_widgets(void) {
  if (brightness_pct_lbl) {
    char b[8];
    snprintf(b, sizeof(b), "%u%%", (unsigned)backlight_pct);
    lv_label_set_text(brightness_pct_lbl, b);
  }
}

static void update_pwr_out_arrows(void) {
  for (int i = 0; i < NUM_PWR_OUT_OPTS; i++) {
    bool on = (i == (int)pwr_out_selection);
    if (pwr_out_arrows[i]) {
      lv_label_set_text(pwr_out_arrows[i], on ? ">" : " ");
      lv_obj_set_style_text_color(pwr_out_arrows[i], on ? COLOR_ACCENT : COLOR_MENU_ACTIVE_GREY, 0);
    }
    if (pwr_out_opt_lbls[i])
      lv_obj_set_style_text_color(pwr_out_opt_lbls[i], on ? COLOR_TEXT : COLOR_MENU_ACTIVE_GREY, 0);
  }
}

static void update_menu_root_arrows(void) {
  for (int i = 0; i < NUM_MENU_ROOT_OPTS; i++) {
    bool on = (i == (int)menu_root_sel);
    if (menu_root_arrows[i]) {
      lv_label_set_text(menu_root_arrows[i], on ? ">" : " ");
      lv_obj_set_style_text_color(menu_root_arrows[i], on ? COLOR_ACCENT : COLOR_MENU_ACTIVE_GREY, 0);
    }
    if (menu_root_opt_lbls[i])
      lv_obj_set_style_text_color(menu_root_opt_lbls[i], on ? COLOR_TEXT : COLOR_MENU_ACTIVE_GREY, 0);
  }
}

static void update_extra_arrows(void) {
  for (int i = 0; i < NUM_EXTRA_OPTS; i++) {
    bool on = (i == (int)menu_extra_sel);
    if (extra_arrows[i]) {
      lv_label_set_text(extra_arrows[i], on ? ">" : " ");
      lv_obj_set_style_text_color(extra_arrows[i], on ? COLOR_ACCENT : COLOR_MENU_ACTIVE_GREY, 0);
    }
    if (extra_opt_lbls[i])
      lv_obj_set_style_text_color(extra_opt_lbls[i], on ? COLOR_TEXT : COLOR_MENU_ACTIVE_GREY, 0);
  }
}

static void update_language_arrows(void) {
  for (int i = 0; i < NUM_LANG_OPTS; i++) {
    bool on = (i == (int)menu_lang_sel);
    if (language_arrows[i]) {
      lv_label_set_text(language_arrows[i], on ? ">" : " ");
      lv_obj_set_style_text_color(language_arrows[i], on ? COLOR_ACCENT : COLOR_MENU_ACTIVE_GREY, 0);
    }
    if (language_opt_lbls[i])
      lv_obj_set_style_text_color(language_opt_lbls[i], on ? COLOR_TEXT : COLOR_MENU_ACTIVE_GREY, 0);
  }
}

static void menu_toast_timer_cb(lv_timer_t *timer) {
  (void)timer;
  if (menu_toast_overlay)
    lv_obj_add_flag(menu_toast_overlay, LV_OBJ_FLAG_HIDDEN);
  menu_toast_timer = NULL;
  if (menu_layer == MENU_LAYER_ROOT && menu_root_overlay)
    lv_obj_move_foreground(menu_root_overlay);
  else if (menu_layer == MENU_LAYER_PWR && pwr_out_overlay)
    lv_obj_move_foreground(pwr_out_overlay);
  else if (menu_layer == MENU_LAYER_EXTRA && extra_overlay)
    lv_obj_move_foreground(extra_overlay);
  else if (menu_layer == MENU_LAYER_BRIGHTNESS && brightness_overlay)
    lv_obj_move_foreground(brightness_overlay);
  else if (menu_layer == MENU_LAYER_LANGUAGE && language_overlay)
    lv_obj_move_foreground(language_overlay);
  update_oil_warning_overlay();
}

static void show_menu_toast(const char *msg, lv_color_t col) {
  if (!menu_toast_overlay || !menu_toast_lbl) return;
  if (menu_toast_timer) {
    lv_timer_delete(menu_toast_timer);
    menu_toast_timer = NULL;
  }
  static char menu_toast_buf[56];
  strncpy(menu_toast_buf, msg, sizeof(menu_toast_buf) - 1u);
  menu_toast_buf[sizeof(menu_toast_buf) - 1u] = '\0';
  lv_label_set_text_static(menu_toast_lbl, menu_toast_buf);
  lv_obj_set_style_text_color(menu_toast_lbl, col, 0);
  lv_obj_clear_flag(menu_toast_overlay, LV_OBJ_FLAG_HIDDEN);
  lv_obj_move_foreground(menu_toast_overlay);
  update_oil_warning_overlay();
  menu_toast_timer = lv_timer_create(menu_toast_timer_cb, 1500, NULL);
  lv_timer_set_repeat_count(menu_toast_timer, 1);
}

static void refresh_extra_active_marks(void) {
  for (int i = 0; i < NUM_EXTRA_OPTS; i++) {
    if (!extra_row_mark_lbls[i]) continue;
    if (extra_feat_on[i]) {
      lv_label_set_text(extra_row_mark_lbls[i], LV_SYMBOL_OK);
      lv_obj_set_style_text_color(extra_row_mark_lbls[i], COLOR_CRUISE_ON, 0);
      lv_obj_clear_flag(extra_row_mark_lbls[i], LV_OBJ_FLAG_HIDDEN);
    } else {
      lv_obj_add_flag(extra_row_mark_lbls[i], LV_OBJ_FLAG_HIDDEN);
    }
  }
}

/** `!` + digit + newline to gateway. Ids 0–2: power → D29–D31; 3–5: extra features → D35–D37. */
static void notify_gateway_epic_button(uint8_t id) {
  Serial2.write('!');
  if (id < 10)
    Serial2.write((char)('0' + id));
  else
    Serial2.write('?');
  Serial2.write('\n');
}

/** `@` + digit + newline; clears matching bit when id is 3–5. */
static void notify_gateway_epic_off(uint8_t id) {
  Serial2.write('@');
  if (id < 10)
    Serial2.write((char)('0' + id));
  else
    Serial2.write('?');
  Serial2.write('\n');
}

static void hide_all_menu_overlays(void) {
  if (menu_root_overlay) lv_obj_add_flag(menu_root_overlay, LV_OBJ_FLAG_HIDDEN);
  if (pwr_out_overlay) lv_obj_add_flag(pwr_out_overlay, LV_OBJ_FLAG_HIDDEN);
  if (extra_overlay) lv_obj_add_flag(extra_overlay, LV_OBJ_FLAG_HIDDEN);
  if (brightness_overlay) lv_obj_add_flag(brightness_overlay, LV_OBJ_FLAG_HIDDEN);
  if (language_overlay) lv_obj_add_flag(language_overlay, LV_OBJ_FLAG_HIDDEN);
}

static void open_menu_root(void) {
  if (!menu_root_overlay) return;
  hide_all_menu_overlays();
  menu_layer = MENU_LAYER_ROOT;
  menu_root_sel = 0;
  update_menu_root_arrows();
  lv_obj_clear_flag(menu_root_overlay, LV_OBJ_FLAG_HIDDEN);
  lv_obj_move_foreground(menu_root_overlay);
}

static bool handle_ui_command(uint8_t cmd) {
  const bool in_menu = (menu_layer != MENU_LAYER_NONE);

  if (in_menu) {
    if (cmd == CMD_QUIT_PWR_OUTPUT) {
      if (menu_layer == MENU_LAYER_ROOT) {
        hide_all_menu_overlays();
        menu_layer = MENU_LAYER_NONE;
      } else {
        if (menu_layer == MENU_LAYER_BRIGHTNESS) {
          /* Final backlight % after leaving menu — flush via loop() like other NVS writes. */
          backlight_nvs_save_after_ms = millis() + 2u;
        }
        menu_layer = MENU_LAYER_ROOT;
        hide_all_menu_overlays();
        if (menu_root_overlay) {
          lv_obj_clear_flag(menu_root_overlay, LV_OBJ_FLAG_HIDDEN);
          lv_obj_move_foreground(menu_root_overlay);
          update_menu_root_arrows();
        }
      }
      return true;
    }

    if (cmd == CMD_NEXT_PAGE) {
      if (menu_layer == MENU_LAYER_ROOT) {
        menu_root_sel = (menu_root_sel + NUM_MENU_ROOT_OPTS - 1) % NUM_MENU_ROOT_OPTS;
        update_menu_root_arrows();
      } else if (menu_layer == MENU_LAYER_PWR) {
        pwr_out_selection = (pwr_out_selection + NUM_PWR_OUT_OPTS - 1) % NUM_PWR_OUT_OPTS;
        update_pwr_out_arrows();
      } else if (menu_layer == MENU_LAYER_EXTRA) {
        menu_extra_sel = (menu_extra_sel + NUM_EXTRA_OPTS - 1) % NUM_EXTRA_OPTS;
        update_extra_arrows();
      } else if (menu_layer == MENU_LAYER_BRIGHTNESS) {
        if (backlight_pct + BACKLIGHT_STEP_PCT <= 100u) {
          backlight_apply_pct((uint8_t)(backlight_pct + BACKLIGHT_STEP_PCT));
          sync_brightness_widgets();
          backlight_schedule_nvs_save();
        }
      } else if (menu_layer == MENU_LAYER_LANGUAGE) {
        menu_lang_sel = (menu_lang_sel + NUM_LANG_OPTS - 1) % NUM_LANG_OPTS;
        update_language_arrows();
      }
      return true;
    }

    if (cmd == CMD_PREV_PAGE) {
      if (menu_layer == MENU_LAYER_ROOT) {
        menu_root_sel = (menu_root_sel + 1) % NUM_MENU_ROOT_OPTS;
        update_menu_root_arrows();
      } else if (menu_layer == MENU_LAYER_PWR) {
        pwr_out_selection = (pwr_out_selection + 1) % NUM_PWR_OUT_OPTS;
        update_pwr_out_arrows();
      } else if (menu_layer == MENU_LAYER_EXTRA) {
        menu_extra_sel = (menu_extra_sel + 1) % NUM_EXTRA_OPTS;
        update_extra_arrows();
      } else if (menu_layer == MENU_LAYER_BRIGHTNESS) {
        if (backlight_pct >= BACKLIGHT_MIN_PCT + BACKLIGHT_STEP_PCT) {
          backlight_apply_pct((uint8_t)(backlight_pct - BACKLIGHT_STEP_PCT));
          sync_brightness_widgets();
          backlight_schedule_nvs_save();
        }
      } else if (menu_layer == MENU_LAYER_LANGUAGE) {
        menu_lang_sel = (menu_lang_sel + 1) % NUM_LANG_OPTS;
        update_language_arrows();
      }
      return true;
    }

    if (cmd == CMD_RESET_TRIP || cmd == CMD_SELECT_PWR) {
      if (menu_layer == MENU_LAYER_ROOT) {
        if (menu_root_sel == 0 && pwr_out_overlay) {
          menu_layer = MENU_LAYER_PWR;
          hide_all_menu_overlays();
          pwr_out_selection = 0;
          update_pwr_out_arrows();
          lv_obj_clear_flag(pwr_out_overlay, LV_OBJ_FLAG_HIDDEN);
          lv_obj_move_foreground(pwr_out_overlay);
        } else if (menu_root_sel == 1 && extra_overlay) {
          menu_layer = MENU_LAYER_EXTRA;
          hide_all_menu_overlays();
          menu_extra_sel = 0;
          update_extra_arrows();
          refresh_extra_active_marks();
          lv_obj_clear_flag(extra_overlay, LV_OBJ_FLAG_HIDDEN);
          lv_obj_move_foreground(extra_overlay);
        } else if (menu_root_sel == 2 && brightness_overlay) {
          menu_layer = MENU_LAYER_BRIGHTNESS;
          hide_all_menu_overlays();
          sync_brightness_widgets();
          lv_obj_clear_flag(brightness_overlay, LV_OBJ_FLAG_HIDDEN);
          lv_obj_move_foreground(brightness_overlay);
        } else if (menu_root_sel == 3 && language_overlay) {
          menu_layer = MENU_LAYER_LANGUAGE;
          hide_all_menu_overlays();
          menu_lang_sel = ui_lang;
          update_language_arrows();
          lv_obj_clear_flag(language_overlay, LV_OBJ_FLAG_HIDDEN);
          lv_obj_move_foreground(language_overlay);
        }
        return true;
      }
      if (menu_layer == MENU_LAYER_PWR) {
        uint8_t sel = pwr_out_selection;
        int8_t prev = pwr_out_active_idx;
        if (prev >= 0 && prev != (int8_t)sel)
          notify_gateway_epic_off((uint8_t)prev);
        notify_gateway_epic_button(sel);
        pwr_out_active_idx = (int8_t)sel;
        apply_shadow_theme(sel);
        char toast[44];
        snprintf(toast, sizeof(toast), "%s ON", pwr_mode_lbl(sel));
        show_menu_toast(toast, COLOR_TOAST_PWR_ON);
        return true;
      }
      if (menu_layer == MENU_LAYER_EXTRA) {
        uint8_t ei = menu_extra_sel;
        char toast[52];
        if (extra_feat_on[ei]) {
          extra_feat_on[ei] = false;
          notify_gateway_epic_off((uint8_t)(3 + ei));
          snprintf(toast, sizeof(toast), "%s OFF", extra_feat_lbl(ei));
          show_menu_toast(toast, COLOR_TOAST_OFF);
        } else {
          extra_feat_on[ei] = true;
          notify_gateway_epic_button((uint8_t)(3 + ei));
          snprintf(toast, sizeof(toast), "%s ON", extra_feat_lbl(ei));
          show_menu_toast(toast, COLOR_TOAST_EXTRA_ON);
        }
        refresh_extra_active_marks();
        return true;
      }
      if (menu_layer == MENU_LAYER_BRIGHTNESS) {
        return true;
      }
      if (menu_layer == MENU_LAYER_LANGUAGE) {
        ui_lang = menu_lang_sel ? (uint8_t)LANG_LV : (uint8_t)LANG_EN;
        ui_lang_schedule_nvs_save();
        apply_ui_language();
        char toast[24];
        snprintf(toast, sizeof(toast), "%s", (ui_lang == LANG_LV) ? "Latviesu" : "English");
        show_menu_toast(toast, COLOR_TOAST_EXTRA_ON);
        return true;
      }
    }

    return false;
  }

  if (main_tabview) {
    if (cmd == CMD_NEXT_PAGE) {
      dash_show_tab((uint8_t)((dash_active_tab + 1u) % NUM_TABS));
      return true;
    } else if (cmd == CMD_PREV_PAGE) {
      dash_show_tab((uint8_t)((dash_active_tab + NUM_TABS - 1u) % NUM_TABS));
      return true;
    } else if (cmd == CMD_RESET_TRIP || cmd == CMD_SELECT_PWR) {
      /* STM32: short reset = `S`, long = `R`; both reset trip on dashboard (Mega-era behavior). */
      reset_trip();
      return true;
    } else if (cmd == CMD_OPEN_PWR_OUTPUT && menu_root_overlay) {
      open_menu_root();
      return true;
    }
  }

  return false;
}

/** Gateway button bytes: handle outside the UART byte-loop so LVGL isn't nested under Serial2 reads. */
#define UI_CMD_RING_CAP 24u
static uint8_t ui_cmd_ring[UI_CMD_RING_CAP];
static uint8_t ui_cmd_head;
static uint8_t ui_cmd_tail;

static void enqueue_ui_cmd(uint8_t cmd) {
  uint8_t n = (uint8_t)((ui_cmd_tail + 1u) % UI_CMD_RING_CAP);
  if (n == ui_cmd_head)
    ui_cmd_head = (uint8_t)((ui_cmd_head + 1u) % UI_CMD_RING_CAP);
  ui_cmd_ring[ui_cmd_tail] = cmd;
  ui_cmd_tail = n;
}

static void drain_ui_cmd_queue(void) {
  for (unsigned guard = 0; guard < UI_CMD_RING_CAP && ui_cmd_head != ui_cmd_tail; guard++) {
    uint8_t cmd = ui_cmd_ring[ui_cmd_head];
    ui_cmd_head = (uint8_t)((ui_cmd_head + 1u) % UI_CMD_RING_CAP);
    handle_ui_command(cmd);
  }
}

static void process_mega_commands(void) {
  const int kMaxSerialBytesPerLoop = 96;
  for (int budget = 0; budget < kMaxSerialBytesPerLoop && Serial2.available(); budget++) {
    int c = Serial2.read();
    if (c < 0) break;
#if DASH_LINK_SNIFFER
    Serial.write((uint8_t)c);
#endif
    uint8_t cmd = (uint8_t)c;

    /* Trip reset `R` vs RPM status `R0`/`R1`: if next byte is 0 or 1, buffer for line parse; else queue reset. */
    if (cmd == (uint8_t)CMD_RESET_TRIP) {
      int p = Serial2.peek();
      if (p == '0' || p == '1') {
        if (mega_line_len < MEGA_LINE_MAX - 1)
          mega_line_buf[mega_line_len++] = (char)cmd;
        else
          mega_line_len = 0;
        continue;
      }
      if (splash_finished && main_tabview)
        enqueue_ui_cmd((uint8_t)CMD_RESET_TRIP);
      continue;
    }

    if (splash_finished && main_tabview) {
      if (cmd == (uint8_t)CMD_NEXT_PAGE || cmd == (uint8_t)CMD_PREV_PAGE ||
          cmd == (uint8_t)CMD_OPEN_PWR_OUTPUT || cmd == (uint8_t)CMD_QUIT_PWR_OUTPUT || cmd == (uint8_t)CMD_SELECT_PWR) {
        enqueue_ui_cmd(cmd);
        continue;
      }
    } else if (handle_ui_command(cmd)) {
      continue;
    }

    if (cmd == '\n' || cmd == '\r') {
      parse_mega_line();
      continue;
    }
    if (mega_line_len < MEGA_LINE_MAX - 1) {
      mega_line_buf[mega_line_len++] = (char)cmd;
    } else {
      mega_line_len = 0;
    }
  }
}

static void update_bottom_bar(void) {
  const int32_t bar_radius = (BOTTOM_BAR_H - 8) / 2;
  const bool active[3] = { check_warning, !oil_ok, cruise_on };

  for (int i = 0; i < 3; i++) {
    if (!bottom_panels[i]) continue;
    lv_obj_set_style_radius(bottom_panels[i], bar_radius, 0);
    lv_obj_set_style_bg_color(bottom_panels[i], COLOR_BAR_NORMAL, 0);
    lv_obj_set_style_bg_opa(bottom_panels[i], LV_OPA_COVER, 0);
    if (active[i]) {
      lv_obj_clear_flag(bottom_panels[i], LV_OBJ_FLAG_HIDDEN);
    } else {
      lv_obj_add_flag(bottom_panels[i], LV_OBJ_FLAG_HIDDEN);
    }
  }

  if (bottom_icons[0] && active[0]) {
    lv_obj_clear_flag(bottom_icons[0], LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_style_text_color(bottom_icons[0], COLOR_CHECK_ACTIVE, 0);
  } else if (bottom_icons[0]) {
    lv_obj_add_flag(bottom_icons[0], LV_OBJ_FLAG_HIDDEN);
  }

  if (bottom_icons[1] && active[1]) {
    lv_obj_clear_flag(bottom_icons[1], LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_style_text_color(bottom_icons[1], COLOR_OIL_WARNING, 0);
  } else if (bottom_icons[1]) {
    lv_obj_add_flag(bottom_icons[1], LV_OBJ_FLAG_HIDDEN);
  }

  if (bottom_icons[2] && active[2]) {
    lv_obj_clear_flag(bottom_icons[2], LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_style_text_color(bottom_icons[2], COLOR_CRUISE_ON, 0);
  } else if (bottom_icons[2]) {
    lv_obj_add_flag(bottom_icons[2], LV_OBJ_FLAG_HIDDEN);
  }

  for (int i = 0; i < 3; i++) {
    if (bottom_captions[i]) lv_obj_add_flag(bottom_captions[i], LV_OBJ_FLAG_HIDDEN);
  }
}

// Easing helper (splash fade)
static inline lv_opa_t ease_smoothstep(uint32_t t256) {
  if (t256 >= 256) return 255;
  uint32_t n = (uint32_t)t256 * t256 * (768 - 2 * t256);
  uint32_t r = n / 65536;
  return (lv_opa_t)(r > 255 ? 255 : r);
}

// Forward declarations for UI build helpers
static void build_main_ui(void);
static void build_main_ui_menus_and_chrome(lv_obj_t *scr);
static void build_ui_dashboard_fuel_trip_accel_power(void);
static void build_ui_dashboard_gauges_gps_aux(void);
static void create_splash(void);

static void style_main_page_bg(lv_obj_t *obj) {
  lv_obj_set_style_bg_color(obj, COLOR_BG, 0);
  lv_obj_set_style_bg_opa(obj, LV_OPA_COVER, 0);
}

/** Half-arc + unit in arc center + large value (shared by RPM, Inj PW, AFR, MAP, CLT, KMH tabs). */
static void build_rpm_style_gauge_page(lv_obj_t *tab, const char *title, const char *unit_center,
                                       lv_obj_t **gauge_out, lv_obj_t **val_lbl_out,
                                       lv_obj_t **title_main_out, lv_obj_t **title_back_out) {
  lv_obj_t *lbl_back = lv_label_create(tab);
  lv_label_set_text(lbl_back, title);
  lv_obj_set_style_text_font(lbl_back, &lv_font_montserrat_20, 0);
  lv_obj_set_style_text_color(lbl_back, lv_color_hex(0x202020), 0);
  lv_obj_align(lbl_back, LV_ALIGN_TOP_MID, 1, 9);
  lv_obj_add_flag(lbl_back, LV_OBJ_FLAG_HIDDEN);

  lv_obj_t *lbl = lv_label_create(tab);
  lv_label_set_text(lbl, title);
  lv_obj_set_style_text_font(lbl, &lv_font_montserrat_20, 0);
  lv_obj_set_style_text_color(lbl, lv_color_hex(0xD8D8D8), 0);
  lv_obj_align(lbl, LV_ALIGN_TOP_MID, 0, 8);
  // Gauge pages: avoid per-frame shadow cost to keep rendering stable.
  if (title_main_out) *title_main_out = lbl;
  if (title_back_out) *title_back_out = lbl_back;

  *gauge_out = lv_arc_create(tab);
  lv_obj_set_size(*gauge_out, 120, 120);
  lv_obj_align(*gauge_out, LV_ALIGN_TOP_MID, 0, 40);
  lv_obj_set_style_bg_opa(*gauge_out, LV_OPA_TRANSP, LV_PART_MAIN);
  lv_obj_set_style_arc_width(*gauge_out, 10, LV_PART_MAIN);
  lv_obj_set_style_arc_color(*gauge_out, lv_color_hex(0x505050), LV_PART_MAIN);
  lv_obj_set_style_arc_width(*gauge_out, 10, LV_PART_INDICATOR);
  lv_obj_set_style_arc_color(*gauge_out, lv_color_hex(0xFFC000), LV_PART_INDICATOR);
  lv_arc_set_rotation(*gauge_out, 0);
  lv_arc_set_bg_angles(*gauge_out, 180, 360);
  lv_arc_set_angles(*gauge_out, 180, 180);
  lv_obj_set_style_opa(*gauge_out, LV_OPA_TRANSP, LV_PART_KNOB);
  lv_obj_set_style_width(*gauge_out, 0, LV_PART_KNOB);
  lv_obj_set_style_height(*gauge_out, 0, LV_PART_KNOB);

  lv_obj_t *unit_lbl = lv_label_create(tab);
  lv_label_set_text(unit_lbl, unit_center);
  lv_obj_set_style_text_color(unit_lbl, COLOR_TEXT, 0);
  lv_obj_set_style_text_font(unit_lbl, &lv_font_montserrat_14, 0);
  lv_obj_align_to(unit_lbl, *gauge_out, LV_ALIGN_CENTER, 0, 6);

  *val_lbl_out = lv_label_create(tab);
  lv_label_set_text(*val_lbl_out, "--");
  lv_obj_set_style_text_color(*val_lbl_out, COLOR_ACCENT, 0);
  lv_obj_set_style_text_font(*val_lbl_out, &lv_font_montserrat_48, 0);
  lv_obj_set_width(*val_lbl_out, TFT_WIDTH - 16);
  lv_obj_set_style_text_align(*val_lbl_out, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align_to(*val_lbl_out, unit_lbl, LV_ALIGN_OUT_BOTTOM_MID, 0, 4);
}

/** File-scope helpers + yield/timer passes during UI build (ESP32 WDT / LVGL stability). */
static void ui_style_menu_overlay(lv_obj_t *ov) {
  lv_obj_set_size(ov, TFT_WIDTH, TFT_HEIGHT);
  lv_obj_align(ov, LV_ALIGN_TOP_LEFT, 0, 0);
  style_main_page_bg(ov);
  lv_obj_set_style_border_width(ov, 0, 0);
  lv_obj_set_style_pad_all(ov, 0, 0);
  lv_obj_remove_flag(ov, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_flag(ov, LV_OBJ_FLAG_HIDDEN);
}

static void ui_add_menu_title(lv_obj_t *ov, const char *title, lv_obj_t **main_out, lv_obj_t **back_out) {
  lv_obj_t *b = lv_label_create(ov);
  lv_label_set_text(b, title);
  lv_obj_set_style_text_font(b, &lv_font_montserrat_24, 0);
  lv_obj_set_style_text_color(b, lv_color_hex(0x202020), 0);
  lv_obj_align(b, LV_ALIGN_TOP_MID, 1, 15);
  lv_obj_add_flag(b, LV_OBJ_FLAG_HIDDEN);

  lv_obj_t *t = lv_label_create(ov);
  lv_label_set_text(t, title);
  lv_obj_set_style_text_font(t, &lv_font_montserrat_24, 0);
  lv_obj_set_style_text_color(t, lv_color_hex(0xD8D8D8), 0);
  lv_obj_align(t, LV_ALIGN_TOP_MID, 0, 14);
  if (main_out) *main_out = t;
  if (back_out) *back_out = b;
}

static lv_obj_t *ui_add_menu_list_row(lv_obj_t *ov, int index, int row_y0, int row_h, int row_dy,
                                      lv_obj_t **arrow_slot, const lv_font_t *font_arrow, lv_obj_t **mark_slot) {
  lv_obj_t *row = lv_obj_create(ov);
  lv_obj_set_size(row, TFT_WIDTH - 24, row_h);
  lv_obj_align(row, LV_ALIGN_TOP_MID, 0, row_y0 + index * row_dy);
  lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(row, 0, 0);
  lv_obj_set_style_radius(row, 0, 0);
  lv_obj_set_style_pad_all(row, 4, 0);
  lv_obj_remove_flag(row, LV_OBJ_FLAG_SCROLLABLE);

  *arrow_slot = lv_label_create(row);
  lv_label_set_text(*arrow_slot, (index == 0) ? ">" : " ");
  lv_obj_set_style_text_font(*arrow_slot, font_arrow, 0);
  lv_obj_set_style_text_color(*arrow_slot, COLOR_ACCENT, 0);
  lv_obj_align(*arrow_slot, LV_ALIGN_LEFT_MID, 4, 0);

  if (mark_slot) {
    *mark_slot = lv_label_create(row);
    lv_label_set_text(*mark_slot, LV_SYMBOL_OK);
    lv_obj_set_style_text_font(*mark_slot, font_arrow, 0);
    lv_obj_set_style_text_color(*mark_slot, COLOR_CRUISE_ON, 0);
    lv_obj_align(*mark_slot, LV_ALIGN_RIGHT_MID, -4, 0);
    lv_obj_add_flag(*mark_slot, LV_OBJ_FLAG_HIDDEN);
  }

  return lv_label_create(row);
}

static void ui_create_aux_grid_overlay(void) {
  if (!main_tabview) return;
  const lv_coord_t cell_h = 22;
  const lv_coord_t grid_h = 2 * cell_h + 6;
  aux_grid_overlay = lv_obj_create(main_tabview);
  lv_obj_set_size(aux_grid_overlay, TFT_WIDTH - 16, grid_h);
  lv_obj_align(aux_grid_overlay, LV_ALIGN_BOTTOM_MID, 0, -1 - PWR_GRID_RAISE_PX);
  lv_obj_set_style_bg_opa(aux_grid_overlay, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(aux_grid_overlay, 0, 0);
  lv_obj_set_style_pad_all(aux_grid_overlay, 0, 0);
  lv_obj_remove_flag(aux_grid_overlay, LV_OBJ_FLAG_SCROLLABLE);

  const lv_coord_t w = TFT_WIDTH - 16;
  const lv_coord_t col_w = (w - 8) / 2;
  const lv_coord_t x_col[2] = { 0, (lv_coord_t)(col_w + 8) };
  const lv_coord_t y_row[2] = { 0, (lv_coord_t)(cell_h + 6) };

  for (int i = 0; i < AUX_SLOT_COUNT; i++) {
    const int r = i / 2;
    const int c = i % 2;

    lv_obj_t *name = lv_label_create(aux_grid_overlay);
    lv_label_set_text(name, "--");
    lv_obj_set_style_text_color(name, COLOR_TEXT, 0);
    lv_obj_set_style_text_font(name, &lv_font_montserrat_14, 0);
    lv_obj_set_pos(name, x_col[c], y_row[r] + 2);
    pwr_name_lbls[i] = name;

    lv_obj_t *val = lv_label_create(aux_grid_overlay);
    lv_label_set_text(val, "--");
    lv_obj_set_style_text_color(val, COLOR_ACCENT, 0);
    lv_obj_set_style_text_font(val, &lv_font_montserrat_14, 0);
    lv_obj_set_width(val, col_w - 28);
    lv_obj_set_style_text_align(val, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_set_pos(val, x_col[c] + 28, y_row[r] + 2);
    power_val_lbl[i + 1] = val;
  }
}

static void build_ui_dashboard_fuel_trip_accel_power(void) {
  lv_obj_t *const tab_fuel = main_tabs[0];
  lv_obj_t *const tab_trip = main_tabs[1];
  lv_obj_t *const tab_accel = main_tabs[2];
  lv_obj_t *const tab_power = main_tabs[3];
  lv_obj_t *lbl;
  lv_obj_t *lbl_back;

  // --- Fuel tab ---
  lbl_back = lv_label_create(tab_fuel);
  lv_label_set_text(lbl_back, "Fuel consumption");
  lv_obj_set_style_text_font(lbl_back, &lv_font_montserrat_20, 0);
  lv_obj_set_style_text_color(lbl_back, lv_color_hex(0x202020), 0);
  lv_obj_align(lbl_back, LV_ALIGN_TOP_MID, 1, 11);
  lv_obj_add_flag(lbl_back, LV_OBJ_FLAG_HIDDEN);
  tab_title_back_lbl[0] = lbl_back;

  lbl = lv_label_create(tab_fuel);
  lv_label_set_text(lbl, "Fuel consumption");
  lv_obj_set_style_text_font(lbl, &lv_font_montserrat_20, 0);
  lv_obj_set_style_text_color(lbl, lv_color_hex(0xD8D8D8), 0);
  lv_obj_align(lbl, LV_ALIGN_TOP_MID, 0, 10);
  apply_title_shadow(lbl);
  tab_title_main_lbl[0] = lbl;

  lv_obj_t *fuel_card = lv_obj_create(tab_fuel);
  lv_obj_set_size(fuel_card, TFT_WIDTH - 30, 110);
  lv_obj_align(fuel_card, LV_ALIGN_CENTER, 0, -5);
  lv_obj_set_style_bg_color(fuel_card, COLOR_CARD, 0);
  lv_obj_set_style_bg_opa(fuel_card, LV_OPA_COVER, 0);
  lv_obj_set_style_radius(fuel_card, 12, 0);
  lv_obj_set_style_border_width(fuel_card, 0, 0);
  lv_obj_set_style_shadow_width(fuel_card, 18, 0);
  lv_obj_set_style_shadow_opa(fuel_card, LV_OPA_80, 0);
  lv_obj_set_style_shadow_color(fuel_card, lv_color_hex(0x000000), 0);
  lv_obj_set_style_shadow_ofs_x(fuel_card, 0, 0);
  lv_obj_set_style_shadow_ofs_y(fuel_card, 6, 0);
  lv_obj_set_style_pad_all(fuel_card, 10, 0);
  lv_obj_remove_flag(fuel_card, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t *fuel_row_inst = lv_obj_create(fuel_card);
  lv_obj_set_size(fuel_row_inst, lv_pct(100), 36);
  lv_obj_align(fuel_row_inst, LV_ALIGN_TOP_MID, 0, 0);
  lv_obj_set_style_bg_color(fuel_row_inst, COLOR_CARD, 0);
  lv_obj_set_style_bg_opa(fuel_row_inst, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(fuel_row_inst, 0, 0);
  lv_obj_set_style_pad_all(fuel_row_inst, 0, 0);
  lv_obj_remove_flag(fuel_row_inst, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t *fuel_inst_lbl = lv_label_create(fuel_row_inst);
  lv_label_set_text(fuel_inst_lbl, "Instant");
  lv_obj_set_style_text_color(fuel_inst_lbl, COLOR_TEXT, 0);
  lv_obj_set_style_text_font(fuel_inst_lbl, &lv_font_montserrat_14, 0);
  lv_obj_align(fuel_inst_lbl, LV_ALIGN_LEFT_MID, 0, 0);
  add_shadow_label(fuel_inst_lbl);
  fuel_inst_name_lbl = fuel_inst_lbl;

  fuel_val_lbl = lv_label_create(fuel_row_inst);
  lv_label_set_text(fuel_val_lbl, "--");
  lv_obj_set_style_text_font(fuel_val_lbl, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(fuel_val_lbl, COLOR_TEXT, 0);
  lv_obj_align(fuel_val_lbl, LV_ALIGN_RIGHT_MID, 0, 0);
  add_shadow_label(fuel_val_lbl);

  lv_obj_t *fuel_row_avg = lv_obj_create(fuel_card);
  lv_obj_set_size(fuel_row_avg, lv_pct(100), 36);
  lv_obj_align(fuel_row_avg, LV_ALIGN_BOTTOM_MID, 0, 0);
  lv_obj_set_style_bg_color(fuel_row_avg, COLOR_CARD, 0);
  lv_obj_set_style_bg_opa(fuel_row_avg, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(fuel_row_avg, 0, 0);
  lv_obj_set_style_pad_all(fuel_row_avg, 0, 0);
  lv_obj_remove_flag(fuel_row_avg, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t *fuel_avg_text = lv_label_create(fuel_row_avg);
  lv_label_set_text(fuel_avg_text, "Average");
  lv_obj_set_style_text_color(fuel_avg_text, COLOR_TEXT, 0);
  lv_obj_set_style_text_font(fuel_avg_text, &lv_font_montserrat_14, 0);
  lv_obj_align(fuel_avg_text, LV_ALIGN_LEFT_MID, 0, 0);
  add_shadow_label(fuel_avg_text);
  fuel_avg_name_lbl = fuel_avg_text;

  fuel_avg_lbl = lv_label_create(fuel_row_avg);
  lv_label_set_text(fuel_avg_lbl, "--");
  lv_obj_set_style_text_font(fuel_avg_lbl, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(fuel_avg_lbl, COLOR_TEXT, 0);
  lv_obj_align(fuel_avg_lbl, LV_ALIGN_RIGHT_MID, 0, 0);
  add_shadow_label(fuel_avg_lbl);

  // --- Trip tab (same centered card + two rows as Fuel tab: Distance / Time) ---
  lbl_back = lv_label_create(tab_trip);
  lv_label_set_text(lbl_back, "Trip computer");
  lv_obj_set_style_text_font(lbl_back, &lv_font_montserrat_20, 0);
  lv_obj_set_style_text_color(lbl_back, lv_color_hex(0x202020), 0);
  lv_obj_align(lbl_back, LV_ALIGN_TOP_MID, 1, 11);
  lv_obj_add_flag(lbl_back, LV_OBJ_FLAG_HIDDEN);
  tab_title_back_lbl[1] = lbl_back;

  lbl = lv_label_create(tab_trip);
  lv_label_set_text(lbl, "Trip computer");
  lv_obj_set_style_text_font(lbl, &lv_font_montserrat_20, 0);
  lv_obj_set_style_text_color(lbl, lv_color_hex(0xD8D8D8), 0);
  lv_obj_align(lbl, LV_ALIGN_TOP_MID, 0, 10);
  apply_title_shadow(lbl);
  tab_title_main_lbl[1] = lbl;

  lv_obj_t *trip_card = lv_obj_create(tab_trip);
  lv_obj_set_size(trip_card, TFT_WIDTH - 30, 110);
  lv_obj_align(trip_card, LV_ALIGN_CENTER, 0, -5);
  lv_obj_set_style_bg_color(trip_card, COLOR_CARD, 0);
  lv_obj_set_style_bg_opa(trip_card, LV_OPA_COVER, 0);
  lv_obj_set_style_radius(trip_card, 12, 0);
  lv_obj_set_style_border_width(trip_card, 0, 0);
  lv_obj_set_style_shadow_width(trip_card, 18, 0);
  lv_obj_set_style_shadow_opa(trip_card, LV_OPA_80, 0);
  lv_obj_set_style_shadow_color(trip_card, lv_color_hex(0x000000), 0);
  lv_obj_set_style_shadow_ofs_x(trip_card, 0, 0);
  lv_obj_set_style_shadow_ofs_y(trip_card, 6, 0);
  lv_obj_set_style_pad_all(trip_card, 10, 0);
  lv_obj_remove_flag(trip_card, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t *trip_row_dist = lv_obj_create(trip_card);
  lv_obj_set_size(trip_row_dist, lv_pct(100), 36);
  lv_obj_align(trip_row_dist, LV_ALIGN_TOP_MID, 0, 0);
  lv_obj_set_style_bg_color(trip_row_dist, COLOR_CARD, 0);
  lv_obj_set_style_bg_opa(trip_row_dist, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(trip_row_dist, 0, 0);
  lv_obj_set_style_pad_all(trip_row_dist, 0, 0);
  lv_obj_remove_flag(trip_row_dist, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t *trip_dist_name = lv_label_create(trip_row_dist);
  lv_label_set_text(trip_dist_name, "Distance");
  lv_obj_set_style_text_color(trip_dist_name, COLOR_TEXT, 0);
  lv_obj_set_style_text_font(trip_dist_name, &lv_font_montserrat_14, 0);
  lv_obj_align(trip_dist_name, LV_ALIGN_LEFT_MID, 0, 0);
  add_shadow_label(trip_dist_name);
  trip_name_lbls[0] = trip_dist_name;

  trip_dist_lbl = lv_label_create(trip_row_dist);
  lv_label_set_text(trip_dist_lbl, "0.0 km");
  lv_obj_set_style_text_font(trip_dist_lbl, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(trip_dist_lbl, COLOR_TEXT, 0);
  lv_obj_align(trip_dist_lbl, LV_ALIGN_RIGHT_MID, 0, 0);
  add_shadow_label(trip_dist_lbl);

  lv_obj_t *trip_row_time = lv_obj_create(trip_card);
  lv_obj_set_size(trip_row_time, lv_pct(100), 36);
  lv_obj_align(trip_row_time, LV_ALIGN_BOTTOM_MID, 0, 0);
  lv_obj_set_style_bg_color(trip_row_time, COLOR_CARD, 0);
  lv_obj_set_style_bg_opa(trip_row_time, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(trip_row_time, 0, 0);
  lv_obj_set_style_pad_all(trip_row_time, 0, 0);
  lv_obj_remove_flag(trip_row_time, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t *trip_time_name = lv_label_create(trip_row_time);
  lv_label_set_text(trip_time_name, "Time");
  lv_obj_set_style_text_color(trip_time_name, COLOR_TEXT, 0);
  lv_obj_set_style_text_font(trip_time_name, &lv_font_montserrat_14, 0);
  lv_obj_align(trip_time_name, LV_ALIGN_LEFT_MID, 0, 0);
  add_shadow_label(trip_time_name);
  trip_name_lbls[1] = trip_time_name;

  trip_time_lbl = lv_label_create(trip_row_time);
  lv_label_set_text(trip_time_lbl, "0:00");
  lv_obj_set_style_text_font(trip_time_lbl, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(trip_time_lbl, COLOR_TEXT, 0);
  lv_obj_align(trip_time_lbl, LV_ALIGN_RIGHT_MID, 0, 0);
  add_shadow_label(trip_time_lbl);

  // --- Acceleration tab (STM32 sends a/b/c/d seconds; same card style as Fuel / Trip) ---
  lbl_back = lv_label_create(tab_accel);
  lv_label_set_text(lbl_back, "Acceleration");
  lv_obj_set_style_text_font(lbl_back, &lv_font_montserrat_20, 0);
  lv_obj_set_style_text_color(lbl_back, lv_color_hex(0x202020), 0);
  lv_obj_align(lbl_back, LV_ALIGN_TOP_MID, 1, 11);
  lv_obj_add_flag(lbl_back, LV_OBJ_FLAG_HIDDEN);
  tab_title_back_lbl[2] = lbl_back;

  lbl = lv_label_create(tab_accel);
  lv_label_set_text(lbl, "Acceleration");
  lv_obj_set_style_text_font(lbl, &lv_font_montserrat_20, 0);
  lv_obj_set_style_text_color(lbl, lv_color_hex(0xD8D8D8), 0);
  lv_obj_align(lbl, LV_ALIGN_TOP_MID, 0, 10);
  apply_title_shadow(lbl);
  tab_title_main_lbl[2] = lbl;

  lv_obj_t *accel_card = lv_obj_create(tab_accel);
  lv_obj_set_size(accel_card, TFT_WIDTH - 30, 164);
  lv_obj_align(accel_card, LV_ALIGN_CENTER, 0, -5);
  lv_obj_set_style_bg_color(accel_card, COLOR_CARD, 0);
  lv_obj_set_style_bg_opa(accel_card, LV_OPA_COVER, 0);
  lv_obj_set_style_radius(accel_card, 12, 0);
  lv_obj_set_style_border_width(accel_card, 0, 0);
  lv_obj_set_style_shadow_width(accel_card, 18, 0);
  lv_obj_set_style_shadow_opa(accel_card, LV_OPA_80, 0);
  lv_obj_set_style_shadow_color(accel_card, lv_color_hex(0x000000), 0);
  lv_obj_set_style_shadow_ofs_x(accel_card, 0, 0);
  lv_obj_set_style_shadow_ofs_y(accel_card, 6, 0);
  lv_obj_set_style_pad_all(accel_card, 10, 0);
  lv_obj_remove_flag(accel_card, LV_OBJ_FLAG_SCROLLABLE);

  static const char *const accel_row_en[4] = { "0-60 km/h", "0-100 km/h", "0-120 km/h", "60-120 km/h" };
  for (int i = 0; i < 4; i++) {
    lv_obj_t *row = lv_obj_create(accel_card);
    lv_obj_set_size(row, lv_pct(100), 36);
    lv_obj_align(row, LV_ALIGN_TOP_MID, 0, i * 36);
    lv_obj_set_style_bg_color(row, COLOR_CARD, 0);
    lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(row, 0, 0);
    lv_obj_set_style_pad_all(row, 0, 0);
    lv_obj_remove_flag(row, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *nl = lv_label_create(row);
    lv_label_set_text(nl, accel_row_en[i]);
    lv_obj_set_style_text_color(nl, COLOR_TEXT, 0);
    lv_obj_set_style_text_font(nl, &lv_font_montserrat_14, 0);
    lv_obj_align(nl, LV_ALIGN_LEFT_MID, 0, 0);
    add_shadow_label(nl);
    accel_name_lbls[i] = nl;

    lv_obj_t *vl = lv_label_create(row);
    lv_label_set_text(vl, "--");
    lv_obj_set_style_text_font(vl, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(vl, COLOR_TEXT, 0);
    lv_obj_align(vl, LV_ALIGN_RIGHT_MID, 0, 0);
    add_shadow_label(vl);
    accel_val_lbls[i] = vl;
  }

  // --- Power tab ---
  lbl_back = lv_label_create(tab_power);
  lv_label_set_text(lbl_back, "Power");
  lv_obj_set_style_text_font(lbl_back, &lv_font_montserrat_20, 0);
  lv_obj_set_style_text_color(lbl_back, lv_color_hex(0x202020), 0);
  lv_obj_align(lbl_back, LV_ALIGN_TOP_MID, 1, 9);
  lv_obj_add_flag(lbl_back, LV_OBJ_FLAG_HIDDEN);
  tab_title_back_lbl[3] = lbl_back;

  lbl = lv_label_create(tab_power);
  lv_label_set_text(lbl, "Power");
  lv_obj_set_style_text_font(lbl, &lv_font_montserrat_20, 0);
  lv_obj_set_style_text_color(lbl, lv_color_hex(0xD8D8D8), 0);
  lv_obj_align(lbl, LV_ALIGN_TOP_MID, 0, 8);
  apply_title_shadow(lbl);
  tab_title_main_lbl[3] = lbl;

  // BOOST gauge
  boost_gauge = lv_arc_create(tab_power);
  lv_obj_set_size(boost_gauge, 120, 120);
  lv_obj_align(boost_gauge, LV_ALIGN_TOP_MID, 0, 40);
  lv_obj_set_style_bg_opa(boost_gauge, LV_OPA_TRANSP, LV_PART_MAIN);
  lv_obj_set_style_arc_width(boost_gauge, 10, LV_PART_MAIN);
  lv_obj_set_style_arc_color(boost_gauge, lv_color_hex(0x505050), LV_PART_MAIN);
  lv_obj_set_style_arc_width(boost_gauge, 10, LV_PART_INDICATOR);
  lv_obj_set_style_arc_color(boost_gauge, lv_color_hex(0xFFC000), LV_PART_INDICATOR);
  lv_arc_set_rotation(boost_gauge, 0);
  lv_arc_set_bg_angles(boost_gauge, 180, 360);
  lv_arc_set_angles(boost_gauge, 180, 180);
  lv_obj_set_style_opa(boost_gauge, LV_OPA_TRANSP, LV_PART_KNOB);
  lv_obj_set_style_width(boost_gauge, 0, LV_PART_KNOB);
  lv_obj_set_style_height(boost_gauge, 0, LV_PART_KNOB);

  lv_obj_t *boost_bar_lbl = lv_label_create(tab_power);
  lv_label_set_text(boost_bar_lbl, "Bar");
  lv_obj_set_style_text_color(boost_bar_lbl, COLOR_TEXT, 0);
  lv_obj_set_style_text_font(boost_bar_lbl, &lv_font_montserrat_14, 0);
  lv_obj_align_to(boost_bar_lbl, boost_gauge, LV_ALIGN_CENTER, 0, 6);
  boost_unit_lbl = boost_bar_lbl;

  boost_val_dot = lv_label_create(tab_power);
  lv_label_set_text(boost_val_dot, ".");
  lv_obj_set_style_text_color(boost_val_dot, COLOR_ACCENT, 0);
  lv_obj_set_style_text_font(boost_val_dot, &lv_font_montserrat_48, 0);
  lv_obj_align_to(boost_val_dot, boost_bar_lbl, LV_ALIGN_OUT_BOTTOM_MID, 0, 4);

  boost_val_left = lv_label_create(tab_power);
  lv_label_set_text(boost_val_left, "--");
  lv_obj_set_style_text_color(boost_val_left, COLOR_ACCENT, 0);
  lv_obj_set_style_text_font(boost_val_left, &lv_font_montserrat_48, 0);
  lv_obj_set_width(boost_val_left, 110);
  lv_obj_set_style_text_align(boost_val_left, LV_TEXT_ALIGN_RIGHT, 0);
  lv_obj_align_to(boost_val_left, boost_val_dot, LV_ALIGN_OUT_LEFT_MID, -8, 0);

  boost_val_right = lv_label_create(tab_power);
  lv_label_set_text(boost_val_right, "--");
  lv_obj_set_style_text_color(boost_val_right, COLOR_ACCENT, 0);
  lv_obj_set_style_text_font(boost_val_right, &lv_font_montserrat_48, 0);
  lv_obj_align_to(boost_val_right, boost_val_dot, LV_ALIGN_OUT_RIGHT_MID, 8, 0);

}

static void build_ui_dashboard_gauges_gps_aux(void) {
  lv_obj_t *const tab_rpm = main_tabs[4];
  lv_obj_t *const tab_inj = main_tabs[5];
  lv_obj_t *const tab_afr = main_tabs[6];
  lv_obj_t *const tab_map = main_tabs[7];
  lv_obj_t *const tab_clt = main_tabs[8];
  lv_obj_t *const tab_kmh = main_tabs[9];
  lv_obj_t *const tab_gps = main_tabs[10];
  build_rpm_style_gauge_page(tab_rpm, "RPM", "RPM", &rpm_gauge, &rpm_val_lbl, &tab_title_main_lbl[4], &tab_title_back_lbl[4]);
  build_rpm_style_gauge_page(tab_inj, "Injector PW", "ms", &inj_gauge, &inj_val_lbl, &tab_title_main_lbl[5], &tab_title_back_lbl[5]);
  build_rpm_style_gauge_page(tab_afr, "AFR", "AFR", &afr_gauge, &afr_val_lbl, &tab_title_main_lbl[6], &tab_title_back_lbl[6]);
  build_rpm_style_gauge_page(tab_map, "MAP", "kPa", &map_gauge, &map_val_lbl, &tab_title_main_lbl[7], &tab_title_back_lbl[7]);
  build_rpm_style_gauge_page(tab_clt, "CLT", "°C", &clt_gauge, &clt_val_lbl, &tab_title_main_lbl[8], &tab_title_back_lbl[8]);
  build_rpm_style_gauge_page(tab_kmh, "KMH", "km/h", &kmh_gauge, &kmh_val_lbl, &tab_title_main_lbl[9], &tab_title_back_lbl[9]);
  {
    lv_obj_t *gps_back = lv_label_create(tab_gps);
    lv_label_set_text(gps_back, "GPS monitor");
    lv_obj_set_style_text_font(gps_back, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(gps_back, lv_color_hex(0x202020), 0);
    lv_obj_align(gps_back, LV_ALIGN_TOP_MID, 1, 9);
    lv_obj_add_flag(gps_back, LV_OBJ_FLAG_HIDDEN);
    tab_title_back_lbl[10] = gps_back;

    lv_obj_t *gps_title = lv_label_create(tab_gps);
    lv_label_set_text(gps_title, "GPS monitor");
    lv_obj_set_style_text_font(gps_title, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(gps_title, lv_color_hex(0xD8D8D8), 0);
    lv_obj_align(gps_title, LV_ALIGN_TOP_MID, 0, 8);
    apply_title_shadow(gps_title);
    tab_title_main_lbl[10] = gps_title;

    for (int i = 0; i < 4; i++) {
      lv_obj_t *row = lv_obj_create(tab_gps);
      lv_obj_set_size(row, TFT_WIDTH - 24, 36);
      lv_obj_align(row, LV_ALIGN_TOP_MID, 0, 46 + i * 44);
      lv_obj_set_style_bg_color(row, COLOR_CARD, 0);
      lv_obj_set_style_bg_opa(row, LV_OPA_COVER, 0);
      lv_obj_set_style_border_width(row, 0, 0);
      lv_obj_set_style_radius(row, 8, 0);
      lv_obj_set_style_pad_all(row, 8, 0);
      lv_obj_remove_flag(row, LV_OBJ_FLAG_SCROLLABLE);

      lv_obj_t *name = lv_label_create(row);
      lv_label_set_text(name, "--");
      lv_obj_set_style_text_font(name, &lv_font_montserrat_14, 0);
      lv_obj_set_style_text_color(name, COLOR_TEXT, 0);
      lv_obj_align(name, LV_ALIGN_LEFT_MID, 0, 0);
      gps_row_name_lbl[i] = name;

      lv_obj_t *val = lv_label_create(row);
      lv_label_set_text(val, "--");
      lv_obj_set_style_text_font(val, &lv_font_montserrat_14, 0);
      lv_obj_set_style_text_color(val, COLOR_ACCENT, 0);
      lv_obj_align(val, LV_ALIGN_RIGHT_MID, 0, 0);
      if (i == 0) gps_link_val_lbl = val;
      if (i == 1) gps_fix_val_lbl = val;
      if (i == 2) gps_sats_val_lbl = val;
      if (i == 3) gps_age_val_lbl = val;
    }
    update_gps_monitor_widgets();
  }

  ui_create_aux_grid_overlay();
  update_aux_sensor_grids();

}

// Build main UI: stacked pages (Fuel … CLT) + bottom indicators — manual tab switch, no lv_tabview.
static void build_main_ui(void) {
  if (main_tabview) return;

  lv_obj_t *scr = lv_scr_act();
  lv_obj_set_style_bg_color(scr, COLOR_BG, 0);
  lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);

  main_tabview = lv_obj_create(scr);
  lv_obj_set_size(main_tabview, TFT_WIDTH, TFT_HEIGHT);
  lv_obj_align(main_tabview, LV_ALIGN_TOP_LEFT, 0, 0);
  lv_obj_set_style_bg_color(main_tabview, COLOR_BG, 0);
  lv_obj_set_style_bg_opa(main_tabview, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(main_tabview, 0, 0);
  lv_obj_set_style_pad_all(main_tabview, 0, 0);
  lv_obj_remove_flag(main_tabview, LV_OBJ_FLAG_SCROLLABLE);

  for (unsigned ti = 0; ti < NUM_TABS; ti++) {
    main_tabs[ti] = lv_obj_create(main_tabview);
    lv_obj_t *tp = main_tabs[ti];
    lv_obj_set_size(tp, TFT_WIDTH, TFT_HEIGHT);
    lv_obj_align(tp, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_set_style_border_width(tp, 0, 0);
    lv_obj_set_style_pad_all(tp, 0, 0);
    lv_obj_remove_flag(tp, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(tp, LV_SCROLLBAR_MODE_OFF);
    style_main_page_bg(tp);
    if (ti != 0)
      lv_obj_add_flag(tp, LV_OBJ_FLAG_HIDDEN);
  }

  build_ui_dashboard_fuel_trip_accel_power();
  for (int z = 0; z < SPLASH_HANDLER_LOOPS; z++)
    lv_timer_handler();
  build_ui_dashboard_gauges_gps_aux();

  dash_show_tab(0);
  build_main_ui_menus_and_chrome(scr);
}

static void build_main_ui_menus_and_chrome(lv_obj_t *scr) {
  lv_obj_t *lbl;
  const int menu_root_row_y0 = 50;
  const int menu_root_row_h  = 44;
  const int menu_root_row_dy = 52;
  const int menu_sub_row_y0  = 58;
  const int menu_sub_row_h   = 48;
  const int menu_sub_row_dy  = 58;

  // Root menu: Power output | Extra features | Brightness | Language (gateway `O` opens this)
  menu_root_overlay = lv_obj_create(scr);
  ui_style_menu_overlay(menu_root_overlay);
  ui_add_menu_title(menu_root_overlay, "Menu", &menu_root_title_main, &menu_root_title_back);
  {
    for (int i = 0; i < NUM_MENU_ROOT_OPTS; i++) {
      lv_obj_t *opt_lbl =
        ui_add_menu_list_row(menu_root_overlay, i, menu_root_row_y0, menu_root_row_h, menu_root_row_dy,
                          &menu_root_arrows[i], &lv_font_montserrat_20, NULL);
      lv_label_set_text(opt_lbl, k_root_menu_en[i]);
      lv_obj_set_style_text_font(opt_lbl, &lv_font_montserrat_20, 0);
      lv_obj_align(opt_lbl, LV_ALIGN_CENTER, 0, 0);
      menu_root_opt_lbls[i] = opt_lbl;
    }
  }
  update_menu_root_arrows();

  // Power output sub-screen (Low / Medium / All Hail Quattro)
  pwr_out_overlay = lv_obj_create(scr);
  ui_style_menu_overlay(pwr_out_overlay);
  ui_add_menu_title(pwr_out_overlay, "Power output", &pwr_title_main, &pwr_title_back);

  for (int i = 0; i < NUM_PWR_OUT_OPTS; i++) {
    lv_obj_t *opt_lbl =
      ui_add_menu_list_row(pwr_out_overlay, i, menu_sub_row_y0, menu_sub_row_h, menu_sub_row_dy, &pwr_out_arrows[i],
                        &lv_font_montserrat_18, NULL);
    lv_label_set_text(opt_lbl, k_pwr_mode_en[i]);
    lv_obj_set_style_text_font(opt_lbl, &lv_font_montserrat_18, 0);
    lv_obj_align(opt_lbl, LV_ALIGN_CENTER, 0, 0);
    pwr_out_opt_lbls[i] = opt_lbl;
  }
  update_pwr_out_arrows();

  // Extra features sub-screen
  extra_overlay = lv_obj_create(scr);
  ui_style_menu_overlay(extra_overlay);
  ui_add_menu_title(extra_overlay, "Extra features", &extra_title_main, &extra_title_back);
  {
    for (int i = 0; i < NUM_EXTRA_OPTS; i++) {
      lv_obj_t *opt_lbl =
        ui_add_menu_list_row(extra_overlay, i, menu_sub_row_y0, menu_sub_row_h, menu_sub_row_dy, &extra_arrows[i],
                          &lv_font_montserrat_18, &extra_row_mark_lbls[i]);
      lv_label_set_text(opt_lbl, k_extra_feat_en[i]);
      lv_obj_set_style_text_font(opt_lbl, &lv_font_montserrat_18, 0);
      lv_obj_set_width(opt_lbl, 118);
      lv_label_set_long_mode(opt_lbl, LV_LABEL_LONG_MODE_DOTS);
      lv_obj_align(opt_lbl, LV_ALIGN_LEFT_MID, 28, 0);
      extra_opt_lbls[i] = opt_lbl;
    }
  }
  update_extra_arrows();

  /* Brightness: label only (no lv_bar). On this firmware + LVGL 9 + partial flush, a bar on a full-screen
   * sibling of the tabview correlated with hard lockups when switching to gauge tabs — PWM still works. */
  brightness_overlay = lv_obj_create(scr);
  ui_style_menu_overlay(brightness_overlay);
  ui_add_menu_title(brightness_overlay, "Brightness", &brightness_title_main, &brightness_title_back);
  brightness_pct_lbl = lv_label_create(brightness_overlay);
  lv_label_set_text(brightness_pct_lbl, "100%");
  lv_obj_set_style_text_font(brightness_pct_lbl, &lv_font_montserrat_48, 0);
  lv_obj_set_style_text_color(brightness_pct_lbl, COLOR_ACCENT, 0);
  lv_obj_align(brightness_pct_lbl, LV_ALIGN_CENTER, 0, -8);
  lv_obj_add_flag(brightness_overlay, LV_OBJ_FLAG_HIDDEN);
  sync_brightness_widgets();

  // Language: English / Latvian runtime UI translation.
  language_overlay = lv_obj_create(scr);
  ui_style_menu_overlay(language_overlay);
  ui_add_menu_title(language_overlay, "Language", &language_title_main, &language_title_back);
  for (int i = 0; i < NUM_LANG_OPTS; i++) {
    lv_obj_t *opt_lbl =
      ui_add_menu_list_row(language_overlay, i, menu_sub_row_y0, menu_sub_row_h, menu_sub_row_dy, &language_arrows[i],
                        &lv_font_montserrat_18, NULL);
    lv_label_set_text(opt_lbl, k_lang_opts_en[i]);
    lv_obj_set_style_text_font(opt_lbl, &lv_font_montserrat_18, 0);
    lv_obj_align(opt_lbl, LV_ALIGN_CENTER, 0, 0);
    language_opt_lbls[i] = opt_lbl;
  }
  menu_lang_sel = ui_lang;
  update_language_arrows();
  lv_obj_add_flag(language_overlay, LV_OBJ_FLAG_HIDDEN);

  apply_shadow_theme(shadow_theme);

  // Bottom bar
  bottom_bar = lv_obj_create(scr);
  lv_obj_set_size(bottom_bar, TFT_WIDTH, BOTTOM_BAR_H);
  lv_obj_align(bottom_bar, LV_ALIGN_BOTTOM_LEFT, 0, 0);
  lv_obj_set_style_bg_opa(bottom_bar, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(bottom_bar, 0, 0);
  lv_obj_set_style_pad_ver(bottom_bar, 3, 0);
  lv_obj_set_style_pad_hor(bottom_bar, 3, 0);
  lv_obj_remove_flag(bottom_bar, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_flag(bottom_bar, LV_OBJ_FLAG_OVERFLOW_VISIBLE);
  lv_obj_set_flex_flow(bottom_bar, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(bottom_bar, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

  const char *bar_titles[] = { "Check", "Oil", "Cruise" };
  for (int i = 0; i < 3; i++) {
    lv_obj_t *pan = lv_obj_create(bottom_bar);
    lv_obj_set_flex_grow(pan, 1);
    lv_obj_set_height(pan, BOTTOM_BAR_H - 8);
    lv_obj_set_style_bg_color(pan, COLOR_BAR_NORMAL, 0);
    lv_obj_set_style_bg_opa(pan, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(pan, 0, 0);
    lv_obj_set_style_radius(pan, (BOTTOM_BAR_H - 8) / 2, 0);
    lv_obj_set_style_shadow_width(pan, 12, 0);
    lv_obj_set_style_shadow_opa(pan, LV_OPA_80, 0);
    lv_obj_set_style_shadow_color(pan, lv_color_hex(0x000000), 0);
    lv_obj_set_style_shadow_ofs_x(pan, 0, 0);
    lv_obj_set_style_shadow_ofs_y(pan, 4, 0);
    lv_obj_set_style_pad_all(pan, 3, 0);
    lv_obj_set_style_outline_width(pan, 0, 0);
    lv_obj_remove_flag(pan, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(pan, LV_SCROLLBAR_MODE_OFF);

    const char *icon_txt = (i == 2) ? LV_SYMBOL_PLAY : LV_SYMBOL_WARNING;
    const char *caption_txt = bar_titles[i];

    lv_obj_t *icon = lv_label_create(pan);
    lv_label_set_text(icon, icon_txt);
    lv_obj_set_style_text_color(icon, COLOR_BAR_NORMAL, 0);
    lv_obj_set_style_text_font(icon, &lv_font_montserrat_14, 0);
    lv_obj_align(icon, LV_ALIGN_CENTER, 0, 0);
    bottom_icons[i] = icon;

    lv_obj_t *caption = lv_label_create(pan);
    lv_label_set_text(caption, caption_txt);
    lv_obj_set_style_text_color(caption, COLOR_CRUISE_ON, 0);
    lv_obj_set_style_text_font(caption, &lv_font_montserrat_14, 0);
    lv_obj_align(caption, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_flag(caption, LV_OBJ_FLAG_HIDDEN);

    bottom_panels[i]   = pan;
    bottom_captions[i] = caption;
  }
  update_bottom_bar();

  // Coolant / brake: yellow text above bottom bar
  fluid_warn_container = lv_obj_create(scr);
  lv_obj_set_width(fluid_warn_container, TFT_WIDTH);
  lv_obj_set_height(fluid_warn_container, LV_SIZE_CONTENT);
  lv_obj_set_style_bg_opa(fluid_warn_container, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(fluid_warn_container, 0, 0);
  lv_obj_set_style_pad_all(fluid_warn_container, 0, 0);
  lv_obj_set_style_pad_row(fluid_warn_container, 2, 0);
  lv_obj_set_flex_flow(fluid_warn_container, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(fluid_warn_container,
                        LV_FLEX_ALIGN_CENTER,
                        LV_FLEX_ALIGN_CENTER,
                        LV_FLEX_ALIGN_CENTER);
  lv_obj_remove_flag(fluid_warn_container, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_flag(fluid_warn_container, LV_OBJ_FLAG_HIDDEN);
  lv_obj_align(fluid_warn_container, LV_ALIGN_BOTTOM_MID, 0, -(BOTTOM_BAR_H + 6));

  fluid_coolant_lbl = lv_label_create(fluid_warn_container);
  lv_label_set_text(fluid_coolant_lbl, "Coolant level too low!");
  lv_obj_set_width(fluid_coolant_lbl, TFT_WIDTH - 8);
  // Avoid continuous marquee animation load; clipping is cheaper and more stable on this target.
  lv_label_set_long_mode(fluid_coolant_lbl, LV_LABEL_LONG_MODE_CLIP);
  lv_obj_set_style_text_color(fluid_coolant_lbl, COLOR_FLUID_WARN_TEXT, 0);
  lv_obj_set_style_text_font(fluid_coolant_lbl, &lv_font_montserrat_18, 0);
  lv_obj_set_style_text_align(fluid_coolant_lbl, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_add_flag(fluid_coolant_lbl, LV_OBJ_FLAG_HIDDEN);
  add_shadow_label(fluid_coolant_lbl);

  fluid_brake_lbl = lv_label_create(fluid_warn_container);
  lv_label_set_text(fluid_brake_lbl, "Brake fluid too low!");
  lv_obj_set_width(fluid_brake_lbl, TFT_WIDTH - 8);
  lv_label_set_long_mode(fluid_brake_lbl, LV_LABEL_LONG_MODE_CLIP);
  lv_obj_set_style_text_color(fluid_brake_lbl, COLOR_FLUID_WARN_TEXT, 0);
  lv_obj_set_style_text_font(fluid_brake_lbl, &lv_font_montserrat_18, 0);
  lv_obj_set_style_text_align(fluid_brake_lbl, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_add_flag(fluid_brake_lbl, LV_OBJ_FLAG_HIDDEN);
  add_shadow_label(fluid_brake_lbl);

  update_fluid_warning_strip();
  apply_ui_language();

  // Full-screen toast over dashboard (“Low ON”, extra ON/OFF, etc.) — below oil alert z-order
  menu_toast_overlay = lv_obj_create(scr);
  lv_obj_set_size(menu_toast_overlay, TFT_WIDTH, TFT_HEIGHT);
  lv_obj_align(menu_toast_overlay, LV_ALIGN_TOP_LEFT, 0, 0);
  lv_obj_set_style_bg_color(menu_toast_overlay, lv_color_hex(0x000000), 0);
  lv_obj_set_style_bg_opa(menu_toast_overlay, LV_OPA_70, 0);
  lv_obj_set_style_border_width(menu_toast_overlay, 0, 0);
  lv_obj_set_style_pad_all(menu_toast_overlay, 12, 0);
  lv_obj_remove_flag(menu_toast_overlay, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_flag(menu_toast_overlay, LV_OBJ_FLAG_HIDDEN);

  menu_toast_lbl = lv_label_create(menu_toast_overlay);
  lv_obj_set_width(menu_toast_lbl, TFT_WIDTH - 24);
  lv_label_set_long_mode(menu_toast_lbl, LV_LABEL_LONG_MODE_WRAP);
  lv_obj_set_style_text_font(menu_toast_lbl, &lv_font_montserrat_48, 0);
  lv_obj_set_style_text_align(menu_toast_lbl, LV_TEXT_ALIGN_CENTER, 0);
  lv_label_set_text_static(menu_toast_lbl, "");
  lv_obj_align(menu_toast_lbl, LV_ALIGN_CENTER, 0, 0);

  // Oil warning overlay
  oil_warning_overlay = lv_obj_create(scr);
  lv_obj_set_size(oil_warning_overlay, TFT_WIDTH, TFT_HEIGHT);
  lv_obj_align(oil_warning_overlay, LV_ALIGN_TOP_LEFT, 0, 0);
  lv_obj_set_style_bg_color(oil_warning_overlay, COLOR_OIL_ALERT_BG, 0);
  lv_obj_set_style_bg_opa(oil_warning_overlay, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(oil_warning_overlay, 0, 0);
  lv_obj_set_style_pad_all(oil_warning_overlay, 0, 0);
  lv_obj_remove_flag(oil_warning_overlay, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_flag(oil_warning_overlay, LV_OBJ_FLAG_HIDDEN);

  lbl = lv_label_create(oil_warning_overlay);
  lv_label_set_text(lbl, LV_SYMBOL_WARNING);
  lv_obj_set_style_text_color(lbl, lv_color_hex(0x000000), 0);
  lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, 0);
  lv_obj_align(lbl, LV_ALIGN_CENTER, 0, 0);
  oil_warning_label = lbl;

  update_oil_warning_overlay();
  /* Layout pass after large tree — LVGL deferred work before first full flush. */
  for (int i = 0; i < SPLASH_HANDLER_LOOPS; i++)
    lv_timer_handler();
  lv_obj_invalidate(scr);
}

// Splash screen with Audi rings and titles
static void create_splash(void) {
  lv_obj_t *scr = lv_scr_act();
  lv_obj_set_style_bg_color(scr, lv_color_hex(0x2A2A2A), 0);
  lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);

  splash_cont = lv_obj_create(scr);
  lv_obj_set_size(splash_cont, TFT_WIDTH, TFT_HEIGHT);
  lv_obj_align(splash_cont, LV_ALIGN_TOP_LEFT, 0, 0);
  lv_obj_set_style_bg_color(splash_cont, lv_color_hex(0x2A2A2A), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(splash_cont, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_border_width(splash_cont, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_all(splash_cont, 0, LV_PART_MAIN);
  lv_obj_remove_flag(splash_cont, LV_OBJ_FLAG_SCROLLABLE);
  // Start transparent for a clean fade-in.
  lv_obj_set_style_opa(splash_cont, LV_OPA_TRANSP, LV_PART_MAIN);

  const int ring_r = 28;
  const int ring_w = 5;
  const int cx = TFT_WIDTH / 2;
  const int cy = TFT_HEIGHT / 2 - 30;
  const int dx = 30;

  for (int i = 0; i < 4; i++) {
    int px = cx - ring_r + (i * dx) - (dx * 3) / 2;

    lv_obj_t *shadow = lv_arc_create(splash_cont);
    lv_obj_set_size(shadow, ring_r * 2 + 4, ring_r * 2 + 4);
    lv_obj_set_style_arc_width(shadow, ring_w + 2, LV_PART_MAIN);
    lv_obj_set_style_arc_color(shadow, lv_color_hex(0x050505), LV_PART_MAIN);
    lv_obj_set_style_arc_opa(shadow, LV_OPA_60, LV_PART_MAIN);
    lv_obj_set_style_arc_width(shadow, 0, LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(shadow, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_opa(shadow, LV_OPA_TRANSP, LV_PART_KNOB);
    lv_obj_set_style_width(shadow, 0, LV_PART_KNOB);
    lv_obj_set_style_height(shadow, 0, LV_PART_KNOB);
    lv_arc_set_rotation(shadow, 0);
    lv_arc_set_bg_angles(shadow, 0, 360);
    lv_arc_set_angles(shadow, 0, 360);
    lv_obj_set_pos(shadow, px + 4, cy - ring_r + 2);

    lv_obj_t *ring = lv_arc_create(splash_cont);
    lv_obj_set_size(ring, ring_r * 2, ring_r * 2);
    lv_obj_set_style_arc_width(ring, ring_w, LV_PART_MAIN);
    lv_obj_set_style_arc_color(ring, lv_color_hex(0xD8D8D8), LV_PART_MAIN);
    lv_obj_set_style_arc_width(ring, 0, LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(ring, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_opa(ring, LV_OPA_TRANSP, LV_PART_KNOB);
    lv_obj_set_style_width(ring, 0, LV_PART_KNOB);
    lv_obj_set_style_height(ring, 0, LV_PART_KNOB);
    lv_arc_set_rotation(ring, 0);
    lv_arc_set_bg_angles(ring, 0, 360);
    lv_arc_set_angles(ring, 0, 360);
    lv_obj_set_pos(ring, px, cy - ring_r);
  }

  lv_obj_t *logo_audi_back = lv_label_create(splash_cont);
  lv_label_set_text(logo_audi_back, "Audi ");
  lv_obj_set_style_text_font(logo_audi_back, &lv_font_montserrat_24, 0);
  lv_obj_set_style_text_color(logo_audi_back, lv_color_hex(0x202020), 0);
  lv_obj_align(logo_audi_back, LV_ALIGN_CENTER, -25, 35);
  lv_obj_add_flag(logo_audi_back, LV_OBJ_FLAG_HIDDEN);

  lv_obj_t *logo_audi = lv_label_create(splash_cont);
  lv_label_set_text(logo_audi, "Audi ");
  lv_obj_set_style_text_font(logo_audi, &lv_font_montserrat_24, 0);
  lv_obj_set_style_text_color(logo_audi, lv_color_hex(0xD8D8D8), 0);
  lv_obj_align(logo_audi, LV_ALIGN_CENTER, -26, 34);
  apply_title_shadow(logo_audi);

  lv_obj_t *logo_80_back = lv_label_create(splash_cont);
  lv_label_set_text(logo_80_back, "80");
  lv_obj_set_style_text_font(logo_80_back, &lv_font_montserrat_24, 0);
  lv_obj_set_style_text_color(logo_80_back, lv_color_hex(0x400000), 0);
  lv_obj_align(logo_80_back, LV_ALIGN_CENTER, 35, 35);
  lv_obj_add_flag(logo_80_back, LV_OBJ_FLAG_HIDDEN);

  lv_obj_t *logo_80 = lv_label_create(splash_cont);
  lv_label_set_text(logo_80, "80");
  lv_obj_set_style_text_font(logo_80, &lv_font_montserrat_24, 0);
  lv_obj_set_style_text_color(logo_80, lv_color_hex(0x3A1EC4), 0); // compensate panel channel swap -> appears red
  lv_obj_align(logo_80, LV_ALIGN_CENTER, 34, 34);
  apply_title_shadow(logo_80);

  lv_obj_t *quattro_back = lv_label_create(splash_cont);
  lv_label_set_text(quattro_back, "Quattro");
  lv_obj_set_style_text_font(quattro_back, &lv_font_montserrat_24, 0);
  lv_obj_set_style_text_color(quattro_back, lv_color_hex(0x202020), 0);
  lv_obj_set_style_text_letter_space(quattro_back, 2, 0);
  lv_obj_align(quattro_back, LV_ALIGN_CENTER, 1, 77);
  lv_obj_add_flag(quattro_back, LV_OBJ_FLAG_HIDDEN);

  lv_obj_t *quattro = lv_label_create(splash_cont);
  lv_label_set_text(quattro, "Quattro");
  lv_obj_set_style_text_font(quattro, &lv_font_montserrat_24, 0);
  lv_obj_set_style_text_color(quattro, lv_color_hex(0xD8D8D8), 0);
  lv_obj_set_style_text_letter_space(quattro, 2, 0);
  lv_obj_align(quattro, LV_ALIGN_CENTER, 0, 76);
  apply_title_shadow(quattro);

  splash_start_ms = millis();
}

#if ARDUINO_ARCH_ESP32
/**
 * Bluetooth: return controller + host BSS to the heap when BT is compiled in but unused.
 *
 * WiFi: this sketch never calls `WiFi.begin` / `esp_wifi_init`, so the modem is not used.
 * We intentionally do not include Arduino `WiFi` or `esp_wifi.h` here — both pull large
 * WiFi stacks into the link (~150KB+ flash / ~10KB+ DRAM on this toolchain) and this
 * project is tight on DRAM. To strip WiFi at build time, use ESP-IDF (see sdkconfig.defaults).
 */
static void dash_disable_unused_wireless(void)
{
#if defined(CONFIG_BT_ENABLED) && CONFIG_BT_ENABLED
  /* Classic ESP32 core ships `btInUse()==true`, so initArduino() skips mem release — do it here (before any BT init). */
  (void)esp_bt_mem_release(ESP_BT_MODE_BTDM);
#endif
}
#endif

void setup() {
#if ARDUINO_ARCH_ESP32
  dash_disable_unused_wireless();
#endif
  // USB / UART0: upload and Serial Monitor.
  Serial.begin(115200);
  // Gateway link on UART2 (see MEGA_SER_* pins in file header). RX/TX order is (rxPin, txPin).
#if ARDUINO_ARCH_ESP32
  /* Must be before begin() — Arduino logs [E] HardwareSerial if resized after start. */
  Serial2.setRxBufferSize(1024);
#endif
  Serial2.begin(MEGA_BAUD, SERIAL_8N1, MEGA_SER_RX_PIN, MEGA_SER_TX_PIN);
  pinMode(OIL_PRESSURE_SWITCH_PIN, INPUT_PULLUP);
#if DASH_FLUID_LEVEL_WARNINGS
#if (COOLANT_LEVEL_PIN >= 34)
  pinMode(COOLANT_LEVEL_PIN, INPUT);
#else
  pinMode(COOLANT_LEVEL_PIN, INPUT_PULLUP);
#endif
#if (BRAKE_FLUID_PIN >= 34)
  pinMode(BRAKE_FLUID_PIN, INPUT);
#else
  pinMode(BRAKE_FLUID_PIN, INPUT_PULLUP);
#endif
#endif
  pinMode(BUZZER_PIN, OUTPUT);
  buzzer_set(false);

  // Initialize TFT_eSPI
  tft.init();
  tft.setRotation(0);          // Portrait 240x320; adjust if your panel is rotated
  // Baseline color setup for ST7789 panels.
  tft.invertDisplay(false);
  // LVGL RGB565 buffer byte order vs TFT_eSPI transfer order.
  // ST7789 often needs swapped bytes; without this colors look rainbow/garbled.
  tft.setSwapBytes(true);
  // Use a UI-like dark grey immediately to avoid stark flashes.
  tft.fillScreen(tft.color565(58, 58, 58));

  backlight_hw_init();
  backlight_load_from_nvs();
  ui_lang_load_from_nvs();

  lv_init();

  const size_t buf_size = sizeof(buf1);
  display = lv_display_create(TFT_WIDTH, TFT_HEIGHT);
  lv_display_set_buffers(display, buf1, buf2, buf_size, LV_DISPLAY_RENDER_MODE_PARTIAL);
  lv_display_set_flush_cb(display, my_disp_flush);
  lv_display_set_color_format(display, LV_COLOR_FORMAT_RGB565);
  lv_display_set_default(display);
  // Ensure active LVGL screen starts clean black before splash object is created.
  lv_obj_clean(lv_scr_act());
  lv_obj_set_style_bg_color(lv_scr_act(), lv_color_hex(0x2A2A2A), 0);
  lv_obj_set_style_bg_opa(lv_scr_act(), LV_OPA_COVER, 0);

  create_splash();
  // Render splash immediately once to avoid transient pre-logo frame.
  lv_timer_handler();
}

void loop() {
  uint32_t now = millis();
  static uint32_t last_tick = 0;
  uint32_t elapsed = (last_tick != 0) ? (now - last_tick) : 1;
  if (elapsed > 100) elapsed = 100;
  if (elapsed < 1) elapsed = 1;
  lv_tick_inc((uint32_t)elapsed);
  last_tick = now;

  if (!splash_finished && splash_cont && splash_start_ms != 0) {
    uint32_t elapsed_ms = now - splash_start_ms;
    uint32_t total_ms = 2 * SPLASH_DURATION_MS;

    if (elapsed_ms >= total_ms) {
      lv_obj_delete(splash_cont);
      splash_cont = NULL;
      splash_finished = true;
      for (int i = 0; i < SPLASH_HANDLER_LOOPS; i++)
        lv_timer_handler();
    } else {
      if (elapsed_ms <= SPLASH_DURATION_MS) {
        // Fade-in 0 -> 1 over first phase
        uint32_t t256 = (elapsed_ms * 256) / SPLASH_DURATION_MS;
        if (t256 > 256) t256 = 256;
        lv_obj_set_style_opa(splash_cont, ease_smoothstep(t256), LV_PART_MAIN);
      } else {
        // Fade-out 1 -> 0 over second phase
        uint32_t out_ms = elapsed_ms - SPLASH_DURATION_MS;
        uint32_t t256 = (out_ms * 256) / SPLASH_DURATION_MS;
        if (t256 > 256) t256 = 256;
        lv_opa_t in_opa = ease_smoothstep(t256);
        lv_opa_t out_opa = (lv_opa_t)(255 - in_opa);
        lv_obj_set_style_opa(splash_cont, out_opa, LV_PART_MAIN);
      }
    }
  }

  if (splash_finished && !main_tabview)
    build_main_ui();

  if (!splash_finished && splash_cont) {
    for (int i = 0; i < SPLASH_HANDLER_LOOPS; i++)
      lv_timer_handler();
  } else if (splash_finished && main_tabview) {
    lv_timer_handler();
  }

  process_mega_commands();
  if (splash_finished && main_tabview)
    drain_ui_cmd_queue();
  poll_fluid_level_switches();
  poll_oil_pressure_switch();
  buzzer_tick(now);
  backlight_flush_nvs_if_due(now);
  ui_lang_flush_nvs_if_due(now);

  if (splash_finished)
    lv_timer_handler();

  delay(0);
}