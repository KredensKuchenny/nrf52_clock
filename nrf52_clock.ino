/**************************************************************************
 *  nRF52840 SuperMini - Clock / Thermo-Hygrometer with BLE   (rev. 18)
 *  -----------------------------------------------------------------------
 *  Board core : https://github.com/pdcook/nRFMicro-Arduino-Core
 *               (Tools -> Board -> "SuperMini nRF52840")
 *
 *  Hardware:
 *    - AHT10 temperature/humidity sensor      -> I2C  (addr 0x38)
 *    - SSD1306 128x32 monochrome OLED         -> I2C  (addr 0x3C)
 *    - Push button                            -> D8  (active LOW, internal pull-up)
 *    - LiPo battery                           -> B+   (sensed via internal VDDH/5)
 *
 *  Features:
 *    - 1 Hz hardware clock on RTC2 (8 Hz counter, one COMPARE IRQ/s); full calendar
 *      with correct month lengths and leap years.
 *    - AHT10 temperature + humidity.
 *    - Battery: oversampled + low-pass filtered %, live voltage, charging detect.
 *    - Firmware deep-discharge protection: System OFF at 3.0 V (the board has
 *      no low-voltage cut-off of its own).
 *    - BLE: read current data and set date/time, both in Polish order
 *      DD-MM-YYYY HH:MM:SS; a time change is confirmed by a random code shown
 *      on the screen. Device name is sent via the scan response.
 *    - Button: short press = advertising on/off (your choice); hold 1.5 s = sleep
 *      now (fires while held); a press while asleep just wakes the screen (does
 *      NOT re-advertise - click again to advertise). 30 ms software debounce.
 *    - Advertising: USB = no time limit; battery = max 30 min while idle (resets
 *      after a client disconnects). Sleep stops advertising but keeps an active
 *      connection alive until it drops. Auto sleep below 50% battery; clock keeps
 *      running on RTC2 with the OLED off.
 **************************************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_AHTX0.h>
#include <bluefruit.h>

/* ====================================================================== */
/*  CONFIGURATION                                                         */
/* ====================================================================== */

// ---- OLED display ----
#define SCREEN_WIDTH   128
#define SCREEN_HEIGHT  32
#define OLED_RESET     -1
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ---- Sensor ----
Adafruit_AHTX0 aht;

// ---- Button ----
// "8" = digital pin D8 on this variant. If it does not match your silk label,
// replace with the correct pin / Pn.nn GPIO number. Pressed = LOW (pull-up).
#define BUTTON_PIN          8
#define BTN_DEBOUNCE_MS     30
#define BTN_LONGPRESS_MS    1500

// ---- main-loop wake cadence (tickless idle sleeps the CPU during delay) ----
#define BTN_POLL_MS         15         // while a button is held: poll for release / long-press
#define IDLE_AWAKE_MS       100        // display on: ~10 Hz loop; taps >=100 ms register (OLED dominates power anyway)
#define IDLE_SLEEP_MS       200        // display off: wake the loop less often to save power

// ---- Debug logging over Serial (USB CDC) ----
// 0 = NO Serial at all: Serial is never started (no USB-CDC enumeration), no
//     log strings are built -> a little less current and CPU. Recommended default.
// 1 = enable debug logs over Serial @115200 (for development).
#define ENABLE_SERIAL_LOG   0
#if ENABLE_SERIAL_LOG
  #define LOG_BEGIN(b)     Serial.begin(b)
  #define LOG_PRINT(...)   Serial.print(__VA_ARGS__)
  #define LOG_PRINTLN(...) Serial.println(__VA_ARGS__)
  #define LOG_FLUSH()      Serial.flush()
#else
  #define LOG_BEGIN(b)     ((void)0)
  #define LOG_PRINT(...)   ((void)0)
  #define LOG_PRINTLN(...) ((void)0)
  #define LOG_FLUSH()      ((void)0)
#endif

// ---- Battery / charging ----
// analogReadVDDHDIV5() returns a RAW ADC count for the internal VDDH/5 input.
// Default ADC config: 10-bit (0..1023), 3.6 V full-scale reference.
//   v_in = raw * (ADC_REF / ADC_MAX);   VDDH = v_in * 5  (undo internal /5)
#define ADC_REF             3.6f
#define ADC_MAX             1024.0f
#define VDDH_DIVIDER        5.0f
#define BATT_CAL            0.9831f    // calibration: 4.060 V (DMM) / 4.13 V (shown)

#define BATT_FULL_V         4.20f      // 100% gauge point
#define BATT_EMPTY_V        3.00f      // 0% gauge point (aligned with protection)
#define BATT_CHARGING_V     4.25f      // fallback charge detect (USB/VBUS is primary)

// Firmware deep-discharge protection. The SuperMini board has NO low-voltage
// cut-off (its charger IC only charges; no protection IC on board), so we
// protect the LiPo cell ourselves: when running on battery and VDDH stays
// at/below BATT_PROTECT_V for BATT_PROTECT_COUNT consecutive readings, the
// chip enters System OFF (deep sleep, ~uA). It wakes only on a button press,
// which resets the MCU - so the clock is lost and must be re-set over BLE.
#define BATT_PROTECT_V      3.00f      // [V] enter System OFF below this
#define BATT_PROTECT_COUNT  3          // consecutive low readings (x BATTERY_PERIOD_S)

#define BATT_OVERSAMPLE     16         // ADC samples averaged per reading
#define BATT_EMA_ALPHA      0.25f      // low-pass factor (smaller = smoother/slower)

#define SLEEP_BATT_THRESHOLD 50        // [%] below this the device sleeps
#define WAKE_WINDOW_MS       (5UL * 60UL * 1000UL)   // button wakes device for 5 min
#define ADV_BATT_WINDOW_MS   (30UL * 60UL * 1000UL)  // on battery: advertise max 30 min while idle

// ---- Periodic task periods (seconds) ----
#define SENSOR_PERIOD_S       2        // on USB: read AHT every 2 s
#define SENSOR_PERIOD_BATT_S  60       // on battery: read AHT every 60 s (save power)
#define SENSOR_RETRY_S        5        // while faulty: probe every 5 s so recovery shows fast
#define BATTERY_PERIOD_S    5
#define LOG_PERIOD_S        10

// ---- OLED night dimming (local time) ----
#define OLED_CONTRAST_DAY    0x80      // day brightness
#define OLED_CONTRAST_NIGHT  0x00      // night brightness
#define NIGHT_START_H        20        // night window = 22:00 .. 06:00
#define NIGHT_END_H          8

// ---- BLE TX power (dBm) ----
#define TXPOWER_USB          4         // on USB
#define TXPOWER_BATT        -4         // on battery (lower = less drain, shorter range)

// ---- BLE confirm code lifetime ----
#define CONFIRM_TIMEOUT_MS  60000UL

/* ====================================================================== */
/*  BLE UUIDs (custom 128-bit, arrays are LSB-first for Bluefruit)         */
/*  Readable form: 0F1E2D3C-4B5A-6978-8796-A5B4C3D2E1xx                     */
/* ====================================================================== */
const uint8_t UUID_CLOCK_SVC[16] = {0x00,0xE1,0xD2,0xC3,0xB4,0xA5,0x96,0x87,0x78,0x69,0x5A,0x4B,0x3C,0x2D,0x1E,0x0F};
const uint8_t UUID_DATA_CHR[16]  = {0x01,0xE1,0xD2,0xC3,0xB4,0xA5,0x96,0x87,0x78,0x69,0x5A,0x4B,0x3C,0x2D,0x1E,0x0F};
const uint8_t UUID_SET_CHR[16]   = {0x02,0xE1,0xD2,0xC3,0xB4,0xA5,0x96,0x87,0x78,0x69,0x5A,0x4B,0x3C,0x2D,0x1E,0x0F};
const uint8_t UUID_CONF_CHR[16]  = {0x03,0xE1,0xD2,0xC3,0xB4,0xA5,0x96,0x87,0x78,0x69,0x5A,0x4B,0x3C,0x2D,0x1E,0x0F};

BLEService        clockService(UUID_CLOCK_SVC);
BLECharacteristic dataChar(UUID_DATA_CHR);
BLECharacteristic setTimeChar(UUID_SET_CHR);
BLECharacteristic confirmChar(UUID_CONF_CHR);
BLEDis            bledis;

/* ====================================================================== */
/*  GLOBAL STATE                                                          */
/* ====================================================================== */
volatile uint32_t g_epoch    = 0;       // seconds since 2000-01-01 00:00:00
volatile bool     g_tickFlag = false;   // set by RTC2 ISR once per second
uint32_t          g_sec      = 0;       // scheduling counter

float g_tempC   = 0.0f;
float g_hum     = 0.0f;
bool  g_sensorOk = false;   // AHT health: set by aht.begin() and every readSensor()
float g_vddh    = 0.0f;
float g_vddhFilt = 0.0f;
bool  g_battInit = false;
int   g_battPct = 0;
bool  g_charging = false;
uint8_t g_lowVoltCount = 0;             // consecutive readings below the protect level

bool     g_displayOff = false;
bool     g_forceSleep = false;          // one-shot manual sleep request (long press), cleared on wake
uint32_t g_wakeUntil  = 0;
bool     g_needDraw   = true;
uint8_t  g_curContrast = 0xFF;          // current OLED contrast (0xFF forces first apply)

bool     g_bleOn      = false;          // advertising intent (also "BT" on the OLED)
uint32_t g_advUntil   = 0;              // on battery: stop advertising at this time when idle
uint16_t g_connHandle = BLE_CONN_HANDLE_INVALID;
int8_t   g_rssi       = 0;              // last RSSI of the active connection (dBm)

volatile bool g_pendingValid = false;
uint32_t      g_pendingEpoch = 0;
uint16_t      g_pendingCode  = 0;
uint32_t      g_pendingExpiryMs = 0;

volatile bool g_buttonIrq = false;      // set by the button interrupt (any press)
volatile bool g_justWoke  = false;      // set when a button press wakes the device

/* ====================================================================== */
/*  CALENDAR HELPERS (leap years, variable month length)                  */
/* ====================================================================== */
static const uint8_t DAYS_IN_MONTH[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
static const char*   WEEKDAY_NAME[7]   = {"Sun","Mon","Tue","Wed","Thu","Fri","Sat"};

bool isLeapYear(uint16_t y) {
  return ((y % 4 == 0) && (y % 100 != 0)) || (y % 400 == 0);
}
uint8_t daysInMonth(uint16_t y, uint8_t m) {
  if (m == 2 && isLeapYear(y)) return 29;
  return DAYS_IN_MONTH[m - 1];
}
uint32_t toEpoch(uint16_t year, uint8_t month, uint8_t day,
                 uint8_t hh, uint8_t mm, uint8_t ss) {
  uint32_t days = 0;
  for (uint16_t y = 2000; y < year; y++)  days += isLeapYear(y) ? 366 : 365;
  for (uint8_t  m = 1;    m < month; m++) days += daysInMonth(year, m);
  days += (uint32_t)(day - 1);
  return (((days * 24UL) + hh) * 60UL + mm) * 60UL + ss;
}
void fromEpoch(uint32_t e, uint16_t &year, uint8_t &month, uint8_t &day,
               uint8_t &hh, uint8_t &mm, uint8_t &ss, uint8_t &wday) {
  ss = e % 60; e /= 60;
  mm = e % 60; e /= 60;
  hh = e % 24; e /= 24;
  uint32_t days = e;                       // days since 2000-01-01 (a Saturday)
  wday = (uint8_t)((days + 6) % 7);        // 0=Sun .. 6=Sat
  uint16_t y = 2000;
  while (true) { uint16_t dy = isLeapYear(y) ? 366 : 365; if (days >= dy) { days -= dy; y++; } else break; }
  year = y;
  uint8_t m = 1;
  while (true) { uint8_t dm = daysInMonth(y, m); if (days >= dm) { days -= dm; m++; } else break; }
  month = m;
  day   = (uint8_t)(days + 1);
}

/* ====================================================================== */
/*  TIME ZONE - Poland (CET/CEST) with automatic DST                      */
/*  g_epoch is stored in UTC; we convert to local Polish time for the     */
/*  display, BLE and logs. Storing UTC (not local) makes DST a clean       */
/*  function of the instant - no ambiguous/repeated hour at the autumn     */
/*  switch, and transitions are handled even if the device was asleep.     */
/*  Rule: CEST (UTC+2) from last Sunday of March 01:00 UTC to last Sunday  */
/*  of October 01:00 UTC; CET (UTC+1) otherwise.                           */
/* ====================================================================== */
uint8_t weekdayOf(uint16_t y, uint8_t mo, uint8_t d) {
  uint32_t days = toEpoch(y, mo, d, 0, 0, 0) / 86400UL;
  return (uint8_t)((days + 6) % 7);        // 0=Sun .. 6=Sat (2000-01-01 was Sat)
}
uint8_t lastSundayDay(uint16_t y, uint8_t mo) {
  uint8_t last = daysInMonth(y, mo);
  return (uint8_t)(last - weekdayOf(y, mo, last));   // step back to Sunday (0=Sun)
}
// Offset (hours) for a UTC instant: 2 = CEST (summer), 1 = CET (winter).
int plOffsetH(uint32_t utc) {
  uint16_t Y; uint8_t mo, d, hh, mm, ss, wd;
  fromEpoch(utc, Y, mo, d, hh, mm, ss, wd);
  if (mo < 3 || mo > 10) return 1;
  if (mo > 3 && mo < 10) return 2;
  uint8_t ls = lastSundayDay(Y, mo);
  if (mo == 3) return (d > ls || (d == ls && hh >= 1)) ? 2 : 1;   // spring: 01:00 UTC
  return               (d < ls || (d == ls && hh <  1)) ? 2 : 1;   // autumn: 01:00 UTC
}
// UTC epoch -> local Polish epoch (for display/BLE/logs).
uint32_t plLocal(uint32_t utc) { return utc + (uint32_t)plOffsetH(utc) * 3600UL; }
// Offset for LOCAL wall-clock fields (used when the user sets local time).
// Boundaries in local time: spring 02:00, autumn 03:00.
int plOffsetFromLocalH(uint16_t Y, uint8_t mo, uint8_t d, uint8_t hh) {
  if (mo < 3 || mo > 10) return 1;
  if (mo > 3 && mo < 10) return 2;
  uint8_t ls = lastSundayDay(Y, mo);
  if (mo == 3) return (d > ls || (d == ls && hh >= 2)) ? 2 : 1;
  return               (d < ls || (d == ls && hh <  3)) ? 2 : 1;
}

/* ====================================================================== */
/*  HARDWARE RTC (RTC2) - exact 1 Hz, keeps time during sleep             */
/*  PRESCALER is 12-bit, so the smallest counter rate is 32768/4096 = 8Hz.*/
/*  A 1 Hz counter is impossible, so the counter runs at 8 Hz and a COMPARE*/
/*  event fires once every 8 counts -> exactly one interrupt per second.   */
/* ====================================================================== */
// PRESCALER is only 12-bit (smallest counter rate = 32768/4096 = 8 Hz), so the
// counter cannot tick at 1 Hz directly. The counter free-runs at 8 Hz and CC[0]
// is advanced by 8 counts on every match, giving ONE interrupt per second (no
// software divider, 8x fewer CPU wake-ups than the old 8 Hz TICK approach).
extern "C" void RTC2_IRQHandler(void) {
  if (NRF_RTC2->EVENTS_COMPARE[0]) {
    NRF_RTC2->EVENTS_COMPARE[0] = 0;
    (void)NRF_RTC2->EVENTS_COMPARE[0];                  // read-back flush (errata-safe)
    NRF_RTC2->CC[0] = (NRF_RTC2->CC[0] + 8) & 0xFFFFFF; // next second (24-bit counter, 8 Hz)
    g_epoch++;
    g_tickFlag = true;
  }
}
void setupRTC2(void) {
  NRF_RTC2->TASKS_STOP  = 1;
  NRF_RTC2->TASKS_CLEAR = 1;
  NRF_RTC2->PRESCALER   = 4095;                       // 32768/(4095+1) = 8 Hz counter
  NRF_RTC2->CC[0]       = 8;                           // first match 8 counts = 1 s later
  NRF_RTC2->EVTENSET    = RTC_EVTENSET_COMPARE0_Msk;   // enable COMPARE0 event
  NRF_RTC2->INTENSET    = RTC_INTENSET_COMPARE0_Msk;   // enable COMPARE0 interrupt
  NVIC_SetPriority(RTC2_IRQn, 7);                      // lowest prio: safe with SoftDevice
  NVIC_ClearPendingIRQ(RTC2_IRQn);
  NVIC_EnableIRQ(RTC2_IRQn);
  NRF_RTC2->TASKS_START = 1;
  LOG_PRINTLN("RTC2 started (1 Hz via COMPARE).");
}

/* ====================================================================== */
/*  BATTERY (oversampled + low-pass filtered)                             */
/* ====================================================================== */
// Reliable external-power detection via the USB VBUS comparator. This works
// even on boards where VDDH does not rise to 5 V while charging (e.g. with an
// on-board LDO / power path), and also covers "no battery, USB only" testing.
bool usbConnected(void) {
  return (NRF_POWER->USBREGSTATUS & POWER_USBREGSTATUS_VBUSDETECT_Msk) != 0;
}

int batteryPercent(float v) {
  if (v >= BATT_CHARGING_V) return 100;   // charging: cannot measure real %
  float pct = (v - BATT_EMPTY_V) / (BATT_FULL_V - BATT_EMPTY_V) * 100.0f;
  if (pct < 0)   pct = 0;
  if (pct > 100) pct = 100;
  return (int)(pct + 0.5f);
}
void readBattery(void) {
  uint32_t acc = 0;
  for (int i = 0; i < BATT_OVERSAMPLE; i++) acc += analogReadVDDHDIV5();
  float rawAvg = (float)acc / (float)BATT_OVERSAMPLE;
  float v = rawAvg * (ADC_REF / ADC_MAX) * VDDH_DIVIDER * BATT_CAL;

  bool usb = usbConnected();
  static bool usbPrev = false;
  // Seed the filter on first run, and RE-SEED whenever USB is plugged/unplugged:
  // the supply rail steps at that moment, so we snap to a fresh reading instead
  // of letting the slow EMA drift (which would keep "CHG" up after unplugging).
  if (!g_battInit || usb != usbPrev) {
    g_vddhFilt = v;
    g_battInit = true;
    usbPrev    = usb;
  } else {
    g_vddhFilt += BATT_EMA_ALPHA * (v - g_vddhFilt);
  }

  g_vddh     = g_vddhFilt;
  // External power = USB present (reliable) OR VDDH clearly above a full cell.
  g_charging = usb || (g_vddh > BATT_CHARGING_V);

  int p = batteryPercent(g_vddh);
  // Hysteresis: only move the displayed % when it changes by >= 2 (stops flicker)
  if (g_charging || g_battPct == 0 || abs(p - g_battPct) >= 2) g_battPct = p;

  LOG_PRINT("Battery: VDDH="); LOG_PRINT(g_vddh, 2);
  LOG_PRINT("V pct=");         LOG_PRINT(g_battPct);
  LOG_PRINT("% charging=");    LOG_PRINTLN(g_charging ? 1 : 0);
}

/* ====================================================================== */
/*  SENSOR                                                                */
/* ====================================================================== */
void readSensor(void) {
  sensors_event_t humidity, temp;
  // Treat the sensor as healthy only if the read succeeds AND the values are
  // physically plausible (a missing/garbled AHT can still return stale bytes).
  if (aht.getEvent(&humidity, &temp) &&
      humidity.relative_humidity >= 0.0f && humidity.relative_humidity <= 100.0f &&
      temp.temperature > -40.0f && temp.temperature < 85.0f) {
    g_tempC = temp.temperature;
    g_hum   = humidity.relative_humidity;
    g_sensorOk = true;
    LOG_PRINT("Sensor: T="); LOG_PRINT(g_tempC, 2);
    LOG_PRINT("C H=");       LOG_PRINT(g_hum, 2);
    LOG_PRINTLN("%");
  } else {
    g_sensorOk = false;
    LOG_PRINTLN("AHT read failed.");
  }
}

/* ====================================================================== */
/*  DISPLAY                                                               */
/* ====================================================================== */
void printRightAligned(const char* s, int16_t y, uint8_t size) {
  display.setTextSize(size);
  int16_t w = (int16_t)strlen(s) * 6 * size;
  display.setCursor(SCREEN_WIDTH - w, y);
  display.print(s);
}
void printCentered(const char* s, int16_t y, uint8_t size) {
  display.setTextSize(size);
  int16_t w = (int16_t)strlen(s) * 6 * size;
  display.setCursor((SCREEN_WIDTH - w) / 2, y);
  display.print(s);
}

// Layout (128x32), full height:
//   [HH:MM](size2)  :SS            87%   (top-right, line 1: battery % or "CHG")
//                                  3.85V   (top-right, line 2: voltage, only on battery)
//   Mon 06-06-2026                         (y16: weekday + day-month-year)
//   T 23.5C  H 45%                     BT* (y24: temp/hum left, BLE state bottom-right)
void drawMainScreen(void) {
  uint16_t Y; uint8_t mo, d, hh, mm, ss, wd;
  uint32_t e;
  __disable_irq(); e = g_epoch; __enable_irq();
  fromEpoch(plLocal(e), Y, mo, d, hh, mm, ss, wd);   // UTC -> Polish local (CET/CEST)

  // Night dimming: half brightness during 22:00..06:00 (local). Only send the
  // I2C command when the level actually changes.
  uint8_t wantC = (hh >= NIGHT_START_H || hh < NIGHT_END_H) ? OLED_CONTRAST_NIGHT
                                                            : OLED_CONTRAST_DAY;
  if (wantC != g_curContrast) {
    g_curContrast = wantC;
    display.ssd1306_command(SSD1306_SETCONTRAST);
    display.ssd1306_command(wantC);
  }

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // Big HH:MM
  display.setTextSize(2);
  display.setCursor(0, 0);
  char t[6]; snprintf(t, sizeof(t), "%02u:%02u", hh, mm);
  display.print(t);

  // Seconds, small, next to the clock
  display.setTextSize(1);
  display.setCursor(64, 4);
  char sbuf[4]; snprintf(sbuf, sizeof(sbuf), ":%02u", ss);
  display.print(sbuf);

  // Top-right line 1: battery % (or "CHG" while charging).
  char top[8];
  if (g_charging) snprintf(top, sizeof(top), "CHG");
  else            snprintf(top, sizeof(top), "%d%%", g_battPct);
  printRightAligned(top, 0, 1);

  // Top-right line 2: battery voltage - shown ONLY when running on battery
  // (while charging VDDH is the USB rail, so the reading is meaningless).
  if (!g_charging) {
    char vbuf[8]; snprintf(vbuf, sizeof(vbuf), "%.2fV", g_vddh);
    printRightAligned(vbuf, 8, 1);
  }

  // Date row
  display.setTextSize(1);
  display.setCursor(0, 16);
  // weekday name + day-month-year (Polish reading order)
  char dbuf[24]; snprintf(dbuf, sizeof(dbuf), "%s %02u-%02u-%04u", WEEKDAY_NAME[wd], d, mo, Y);
  display.print(dbuf);

  // Temperature + humidity (or a clear fault marker if the AHT is dead).
  display.setCursor(0, 24);
  char hbuf[24];
  if (g_sensorOk) snprintf(hbuf, sizeof(hbuf), "T %.1fC  H %.0f%%", g_tempC, g_hum);
  else            snprintf(hbuf, sizeof(hbuf), "SENSOR ERROR");   // AHT fault
  display.print(hbuf);

  // BLE state, bottom-right corner: nothing / "BT" / "BT*"
  // Drive this off the real connection, not g_bleOn: a connection kept alive
  // through sleep must still show "BT*" even though advertising is off.
  if      (Bluefruit.connected()) printRightAligned("BT*", 24, 1);  // client connected
  else if (g_bleOn)               printRightAligned("BT",  24, 1);  // advertising, idle

  display.display();
}

// Big, centered confirmation code.
void drawConfirmScreen(void) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  printCentered("BLE set time - code:", 2, 1);
  char c[6]; snprintf(c, sizeof(c), "%04u", g_pendingCode);
  printCentered(c, 14, 2);
  display.display();
}

/* ====================================================================== */
/*  TRUE RANDOM (nRF52 hardware RNG)                                      */
/* ====================================================================== */
// Returns a genuinely random 4-digit code (1000..9999).
// The nRF52840 has a hardware True-RNG. While the SoftDevice (BLE) is running
// it owns the RNG peripheral, so we must pull entropy through it with
// sd_rand_application_vector_get() instead of touching NRF_RNG directly.
uint16_t random4digit(void) {
  uint8_t buf[2];
  uint8_t avail = 0;

  // The RNG pool is normally already full, but wait briefly just in case.
  for (int i = 0; i < 50; i++) {
    sd_rand_application_bytes_available_get(&avail);
    if (avail >= sizeof(buf)) break;
    delay(1);
  }

  if (avail >= sizeof(buf) &&
      sd_rand_application_vector_get(buf, sizeof(buf)) == NRF_SUCCESS) {
    uint16_t v = ((uint16_t)buf[0] << 8) | buf[1];   // true-random 0..65535
    return 1000 + (v % 9000);                        // -> 1000..9999
  }

  // Fallback (rare): reseed the PRNG from a floating analog pin + timers.
  // A0 is just an unused analog input used purely as an entropy source.
  randomSeed(((uint32_t)analogRead(A0) << 16) ^ micros() ^ analogReadVDDHDIV5());
  return (uint16_t)random(1000, 10000);
}

/* ====================================================================== */
/*  BLE                                                                   */
/* ====================================================================== */
void updateBleData(void) {
  uint16_t Y; uint8_t mo, d, hh, mm, ss, wd;
  uint32_t e;
  __disable_irq(); e = g_epoch; __enable_irq();
  fromEpoch(plLocal(e), Y, mo, d, hh, mm, ss, wd);   // UTC -> Polish local (CET/CEST)

  char buf[80];
  // Refresh RSSI of the active link (only valid while connected).
  if (g_connHandle != BLE_CONN_HANDLE_INVALID) {
    int8_t r; uint8_t ch;
    if (sd_ble_gap_rssi_get(g_connHandle, &r, &ch) == NRF_SUCCESS) g_rssi = r;
  }
  // Date in Polish reading order: day-month-year (DD-MM-YYYY). R = link RSSI (dBm).
  // S = sensor health (1 = AHT OK, 0 = fault -> T/H are not trustworthy).
  int n = snprintf(buf, sizeof(buf),
                   "%02u-%02u-%04u %02u:%02u:%02u;T=%.1f;H=%.1f;B=%d;V=%.2f;CHG=%d;R=%d;S=%d",
                   d, mo, Y, hh, mm, ss, g_tempC, g_hum, g_battPct, g_vddh, g_charging ? 1 : 0, g_rssi, g_sensorOk ? 1 : 0);
  if (n < 0) return;
  if (n > (int)sizeof(buf) - 1) n = sizeof(buf) - 1;   // clamp (defensive)
  dataChar.write((uint8_t*)buf, n);
  if (Bluefruit.connected()) dataChar.notify((uint8_t*)buf, n);
}

void connectCb(uint16_t conn) {
  g_connHandle = conn;
  sd_ble_gap_rssi_start(conn, 0, 0);   // begin RSSI sampling for the link-quality readout
  LOG_PRINTLN("BLE connected.");
  g_needDraw = true;
}
void disconnectCb(uint16_t conn, uint8_t reason) {
  (void)conn; (void)reason;
  g_connHandle = BLE_CONN_HANDLE_INVALID;
  LOG_PRINT("BLE disconnected, reason=0x");
  LOG_PRINTLN(reason, HEX);
  if (!g_displayOff && g_bleOn) {
    // Awake and BLE still wanted: advertising auto-restarts (restartOnDisconnect
    // is true). Restart the 30-min battery cap from this disconnect moment.
    g_advUntil = millis() + ADV_BATT_WINDOW_MS;
  } else {
    // Asleep, or BLE was turned off: make sure we are NOT advertising.
    Bluefruit.Advertising.restartOnDisconnect(false);
    Bluefruit.Advertising.stop();
    g_bleOn = false;
  }
  g_needDraw = true;
}

void setTimeWriteCb(uint16_t conn, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
  (void)conn; (void)chr;
  char buf[48];
  uint16_t n = (len < sizeof(buf) - 1) ? len : (sizeof(buf) - 1);
  memcpy(buf, data, n); buf[n] = '\0';
  LOG_PRINT("BLE set-time request: "); LOG_PRINTLN(buf);

  int Y, mo, d, hh, mm, ss;
  // Input format: DD-MM-YYYY HH:MM:SS
  if (sscanf(buf, "%d-%d-%d %d:%d:%d", &d, &mo, &Y, &hh, &mm, &ss) == 6 &&
      Y >= 2000 && Y <= 2099 && mo >= 1 && mo <= 12 &&
      d >= 1 && d <= daysInMonth((uint16_t)Y, (uint8_t)mo) &&
      hh >= 0 && hh < 24 && mm >= 0 && mm < 60 && ss >= 0 && ss < 60) {
    // User sends LOCAL Polish time; store UTC so DST is handled automatically.
    g_pendingEpoch    = toEpoch((uint16_t)Y, (uint8_t)mo, (uint8_t)d, (uint8_t)hh, (uint8_t)mm, (uint8_t)ss)
                        - (uint32_t)plOffsetFromLocalH((uint16_t)Y, (uint8_t)mo, (uint8_t)d, (uint8_t)hh) * 3600UL;
    g_pendingCode     = random4digit();              // hardware true-random
    g_pendingExpiryMs = millis() + CONFIRM_TIMEOUT_MS;
    g_pendingValid    = true;
    g_needDraw        = true;
    LOG_PRINT("Confirm code on screen: "); LOG_PRINT(g_pendingCode);
    LOG_PRINTLN("  (write it to CONFIRM within 60 s)");
  } else {
    LOG_PRINTLN("Invalid format. Expected: DD-MM-YYYY HH:MM:SS");
  }
}

void confirmWriteCb(uint16_t conn, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
  (void)conn; (void)chr;
  char buf[16];
  uint16_t n = (len < sizeof(buf) - 1) ? len : (sizeof(buf) - 1);
  memcpy(buf, data, n); buf[n] = '\0';
  int code = atoi(buf);
  LOG_PRINT("BLE confirm received: "); LOG_PRINTLN(code);

  if (!g_pendingValid) { LOG_PRINTLN("No pending request."); return; }
  if ((int32_t)(millis() - g_pendingExpiryMs) >= 0) {
    LOG_PRINTLN("Request expired."); g_pendingValid = false; return;
  }
  if (code == (int)g_pendingCode) {
    __disable_irq();
    g_epoch  = g_pendingEpoch;
    NRF_RTC2->TASKS_CLEAR = 1;            // counter -> 0
    NRF_RTC2->CC[0]       = 8;            // first tick exactly 1 s from now
    __enable_irq();
    g_pendingValid = false;
    g_needDraw = true;
    LOG_PRINTLN("Clock updated via BLE (confirmed).");
  } else {
    LOG_PRINTLN("Wrong confirm code.");
  }
}

void setupBleService(void) {
  clockService.begin();

  dataChar.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  dataChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  dataChar.setMaxLen(80);
  dataChar.setUserDescriptor("Clock data DD-MM-YYYY (read/notify)");
  dataChar.begin();

  setTimeChar.setProperties(CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP);
  setTimeChar.setPermission(SECMODE_NO_ACCESS, SECMODE_OPEN);
  setTimeChar.setMaxLen(48);
  setTimeChar.setUserDescriptor("Set date/time (DD-MM-YYYY HH:MM:SS)");
  setTimeChar.setWriteCallback(setTimeWriteCb);
  setTimeChar.begin();

  confirmChar.setProperties(CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP);
  confirmChar.setPermission(SECMODE_NO_ACCESS, SECMODE_OPEN);
  confirmChar.setMaxLen(16);
  confirmChar.setUserDescriptor("Confirm code (4 digits from screen)");
  confirmChar.setWriteCallback(confirmWriteCb);
  confirmChar.begin();
}

void setupAdv(void) {
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(clockService);
  // The 128-bit UUID fills the advertising packet, so put the name in the
  // scan response - this is why the full "nRF52-Clock" name now shows up.
  Bluefruit.ScanResponse.addName();
  Bluefruit.Advertising.setInterval(32, 244);   // units of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);
}

void bleStart(void) {
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.start(0);                // advertise until connected
  g_bleOn = true;
  g_advUntil = millis() + ADV_BATT_WINDOW_MS;    // arm 30-min cap (ignored on USB)
  g_needDraw = true;
  LOG_PRINTLN("BLE advertising ON.");
}
// Manual OFF (short click while awake): stop advertising AND drop any connection.
void bleStop(void) {
  Bluefruit.Advertising.restartOnDisconnect(false);  // do NOT auto re-advertise
  Bluefruit.Advertising.stop();                       // stop advertising first
  if (g_connHandle != BLE_CONN_HANDLE_INVALID) {
    Bluefruit.disconnect(g_connHandle);               // then drop any connection
  }
  g_bleOn = false;
  g_needDraw = true;
  LOG_PRINTLN("BLE OFF (manual).");
}
// Sleep variant: stop advertising but KEEP an active connection alive. BLE fully
// idles only once that client disconnects (handled in disconnectCb). A wake will
// NOT re-advertise (g_bleOn=false) - the user re-enables it with a short click.
void advStopKeepConn(void) {
  Bluefruit.Advertising.restartOnDisconnect(false);
  Bluefruit.Advertising.stop();
  g_bleOn = false;
  g_needDraw = true;
}

/* ====================================================================== */
/*  BUTTON                                                                */
/* ====================================================================== */
void buttonISR(void) {
  // Fires on every press (falling edge). Latches the event so a quick tap is
  // never missed, and wakes the CPU from low-power sleep.
  g_buttonIrq = true;
}

void wakeNow(void) {
  g_forceSleep = false;           // a wake cancels any pending manual-sleep request
  g_wakeUntil = millis() + WAKE_WINDOW_MS;
  if (g_displayOff) {
    display.ssd1306_command(SSD1306_DISPLAYON);
    g_displayOff = false;
    g_needDraw = true;
    LOG_PRINTLN("Woken by button.");
  }
}

// Enter / leave manual sleep. Called the moment a long press reaches 1.5 s.
void doLongPress(void) {
  // One-shot "sleep now" command (NOT a toggle). The old toggle made every other
  // long press fail: after a manual sleep + wake, g_forceSleep was still true, so
  // the next long press turned it OFF (device stayed awake) instead of sleeping.
  // Now a long press always requests sleep; wakeNow() clears the request on wake.
  g_forceSleep = true;
  g_wakeUntil  = millis();        // close the wake window -> sleep this iteration
  LOG_PRINTLN("Button LONG -> sleep now.");
}

// Polled, debounced button handler.
//  - while asleep: the waking press only turns the screen on (see loop())
//  - while awake : a short release toggles advertising on/off; holding >=1.5 s
//                  sleeps the instant 1.5 s elapses (no need to release first)
void handleButton(void) {
  static bool     prevStable  = HIGH;
  static uint32_t tLastChange = 0;
  static uint32_t tPressStart = 0;
  static bool     longFired   = false;   // long action already fired this press
  static bool     wakePress   = false;   // this press only woke the device

  bool now = digitalRead(BUTTON_PIN);
  uint32_t ms = millis();

  // Edge detection with debounce
  if (now != prevStable && (ms - tLastChange) > BTN_DEBOUNCE_MS) {
    tLastChange = ms;
    prevStable  = now;
    if (now == LOW) {                    // ---- press starts ----
      tPressStart = ms;
      longFired   = false;
      wakePress   = g_justWoke;          // was this the press that woke us?
      g_justWoke  = false;
    } else {                             // ---- released ----
      if (!wakePress && !longFired) {    // genuine short press while awake
        if (g_bleOn) bleStop(); else bleStart();   // toggle advertising on/off
      }
      wakePress = false;
      longFired = false;
    }
  }

  // Fire the long-press action AS SOON AS 1.5 s of holding elapses.
  if (prevStable == LOW && !wakePress && !longFired &&
      (ms - tPressStart) >= BTN_LONGPRESS_MS) {
    longFired = true;
    doLongPress();
  }

  // Safety: if the button is idle (released) drop any stale wake flag so the
  // next real press is never mistaken for a wake press.
  if (now == HIGH) g_justWoke = false;
}

/* ====================================================================== */
/*  POWER MANAGEMENT                                                      */
/* ====================================================================== */

// Deep-discharge protection: enter System OFF to stop draining the cell.
// On wake (button press) the chip RESETS, so RAM and the clock are lost.
void enterSystemOff(void) {
  LOG_PRINTLN("CRITICAL: battery low -> entering System OFF to protect the cell.");
  LOG_FLUSH();

  // Show a final notice, then power the panel down.
  display.ssd1306_command(SSD1306_DISPLAYON);
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  printCentered("LOW BATTERY", 4, 1);
  printCentered("powering off", 16, 1);
  display.display();
  delay(2500);
  display.ssd1306_command(SSD1306_DISPLAYOFF);

  // Stop the BLE radio.
  Bluefruit.Advertising.stop();

  // Arm the button as a wake-up source: SENSE = LOW so a press wakes the MCU
  // from System OFF (which then boots from setup() again).
  // g_ADigitalPinMap[] maps the Arduino pin to the absolute nRF GPIO (0..47).
  // If your core does not provide it, replace `pin` with D8's raw nRF pin.
  uint32_t pin = g_ADigitalPinMap[BUTTON_PIN];
  NRF_GPIO_Type* port = (pin < 32) ? NRF_P0 : NRF_P1;
  uint32_t pn = pin & 31;
  port->PIN_CNF[pn] =
      ((uint32_t)GPIO_PIN_CNF_DIR_Input     << GPIO_PIN_CNF_DIR_Pos)   |
      ((uint32_t)GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
      ((uint32_t)GPIO_PIN_CNF_PULL_Pullup   << GPIO_PIN_CNF_PULL_Pos)  |
      ((uint32_t)GPIO_PIN_CNF_DRIVE_S0S1    << GPIO_PIN_CNF_DRIVE_Pos) |
      ((uint32_t)GPIO_PIN_CNF_SENSE_Low     << GPIO_PIN_CNF_SENSE_Pos);

  // Enter System OFF (SoftDevice-safe call). Never returns in normal operation;
  // with a debugger attached it is emulated and may return, so loop.
  for (;;) {
    sd_power_system_off();
    __WFE();
  }
}

// Called after each battery reading. Triggers protection only on battery
// (never while charging) and only after several consecutive low readings,
// so a brief voltage dip (e.g. radio TX spike) cannot shut the device down.
void checkBatteryProtection(void) {
  if (!g_battInit) return;
  if (!g_charging && g_vddh <= BATT_PROTECT_V) {
    if (++g_lowVoltCount >= BATT_PROTECT_COUNT) enterSystemOff();
  } else {
    g_lowVoltCount = 0;
  }
}

void enterSleep(void) {
  if (!g_displayOff) {
    display.ssd1306_command(SSD1306_DISPLAYOFF);
    g_displayOff = true;
    advStopKeepConn();   // stop advertising; an active connection stays until it drops
    LOG_PRINT("Sleep (display off). Reason: battPct="); LOG_PRINT(g_battPct);
    LOG_PRINT(" vddh=");        LOG_PRINT(g_vddh, 2);
    LOG_PRINT(" charging=");    LOG_PRINT(g_charging ? 1 : 0);
    LOG_PRINT(" forceSleep=");  LOG_PRINTLN(g_forceSleep ? 1 : 0);
    LOG_FLUSH();
  }
  // Idle waiting is handled by the unified low-power delay at the end of loop().
}

/* ====================================================================== */
/*  STATUS LOG                                                            */
/* ====================================================================== */
void logStatus(void) {
#if ENABLE_SERIAL_LOG
  uint16_t Y; uint8_t mo, d, hh, mm, ss, wd;
  uint32_t e;
  __disable_irq(); e = g_epoch; __enable_irq();
  fromEpoch(plLocal(e), Y, mo, d, hh, mm, ss, wd);   // UTC -> Polish local (CET/CEST)
  char line[96];
  // Date in Polish reading order: day-month-year (DD-MM-YYYY).
  snprintf(line, sizeof(line),
           "[%02u-%02u-%04u %02u:%02u:%02u] T=%.1fC H=%.0f%% Batt=%d%% Chg=%d Force=%d BLE=%d Sleep=%d",
           d, mo, Y, hh, mm, ss, g_tempC, g_hum, g_battPct,
           g_charging ? 1 : 0, g_forceSleep ? 1 : 0, g_bleOn ? 1 : 0, g_displayOff ? 1 : 0);
  LOG_PRINTLN(line);
#endif
}

/* ====================================================================== */
/*  SETUP                                                                 */
/* ====================================================================== */
void setup(void) {
  LOG_BEGIN(115200);
#if ENABLE_SERIAL_LOG
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0 < 2000)) delay(10);
#endif
  LOG_PRINTLN();
  LOG_PRINTLN("=== nRF52840 SuperMini Clock (rev.18) ===");

  // Let the I2C peripherals (OLED, AHT) power up before we init them. Previously
  // the up-to-2 s "wait for Serial" accidentally provided this settle time; with
  // Serial logging off (ENABLE_SERIAL_LOG=0) that wait is gone, so without this
  // explicit delay the SSD1306 can miss its init sequence and stay blank.
  delay(250);

  Wire.begin();

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    LOG_PRINTLN("SSD1306 init FAILED.");
  } else {
    LOG_PRINTLN("SSD1306 OK.");
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("Booting...");
    display.display();
  }

  if (!aht.begin()) { g_sensorOk = false; LOG_PRINTLN("AHT10/20 NOT found."); }
  else              {                      LOG_PRINTLN("AHT10/20 OK."); }   // readSensor() below confirms

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonISR, FALLING);

  // Max bandwidth -> ATT MTU 247 instead of the default 23. Without this a
  // NOTIFY packet can only carry MTU-3 = 20 bytes, which truncates the ~44-byte
  // data string (temp/hum/batt get cut off). MUST be called before begin().
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
  Bluefruit.begin();                 // starts SoftDevice + LFCLK (needed by RTC2)
  Bluefruit.setTxPower(usbConnected() ? TXPOWER_USB : TXPOWER_BATT);
  Bluefruit.setName("nRF52-Clock");
  Bluefruit.Periph.setConnectCallback(connectCb);
  Bluefruit.Periph.setDisconnectCallback(disconnectCb);

  bledis.setManufacturer("_KrEdEnS_");
  bledis.setModel("SuperMini Clock");
  bledis.begin();

  setupBleService();
  setupAdv();
  bleStart();                        // BLE on by default

  g_epoch = toEpoch(2025, 1, 1, 0, 0, 0) - 3600;   // default (UTC; local shows 2025-01-01 00:00 CET)
  setupRTC2();

  // Seed the fallback PRNG from a floating analog pin + timer (the confirm code
  // itself uses the hardware TRNG; this seed only matters if that ever fails).
  randomSeed(((uint32_t)analogRead(A0) << 16) ^ micros() ^ analogReadVDDHDIV5());

  readBattery();
  readSensor();

  g_wakeUntil = millis() + WAKE_WINDOW_MS;
  g_needDraw  = true;
  LOG_PRINTLN("Setup done.");
}

/* ====================================================================== */
/*  MAIN LOOP                                                             */
/* ====================================================================== */
void loop(void) {
  // 1) Instant wake on a single click (interrupt-latched, no need to hold).
  if (g_buttonIrq) {
    g_buttonIrq = false;
    if (g_displayOff) { wakeNow(); g_justWoke = true; }
  }

  // 2) Classify presses while awake (BLE toggle / forced sleep).
  handleButton();

  // 2b) React promptly to USB plug/unplug (don't wait for the 5 s battery poll).
  {
    static bool usbWas = false;
    bool usbNow = usbConnected();
    if (usbNow != usbWas) {
      usbWas = usbNow;
      readBattery(); g_needDraw = true;
      Bluefruit.setTxPower(usbNow ? TXPOWER_USB : TXPOWER_BATT);   // save power on battery
      // Unplugged -> we're on battery now: start the 30-min advertising cap fresh.
      if (!usbNow && g_bleOn) g_advUntil = millis() + ADV_BATT_WINDOW_MS;
    }
  }

  // 2c) Battery-only advertising cap: after 30 min idle (nobody connected) on
  //     battery, stop advertising to save power. On USB there is no cap. The user
  //     re-enables advertising (and resets the 30 min) with a short click.
  if (!g_displayOff && g_bleOn && !g_charging &&
      g_connHandle == BLE_CONN_HANDLE_INVALID &&
      (int32_t)(millis() - g_advUntil) >= 0) {
    Bluefruit.Advertising.restartOnDisconnect(false);
    Bluefruit.Advertising.stop();
    g_bleOn = false;
    g_needDraw = true;
    LOG_PRINTLN("BLE advertising auto-off (30 min idle, battery).");
  }

  // 3) 1 Hz tasks driven by the RTC2 tick.
  if (g_tickFlag) {
    g_tickFlag = false;
    g_sec++;
    if (g_sec % BATTERY_PERIOD_S == 0) { readBattery(); checkBatteryProtection(); }
    // Read the sensor while awake, OR while asleep ONLY if a client is connected
    // (so a connected logger gets fresh data, but a sleeping idle device doesn't
    // waste power waking for I2C). Rate still depends on power: 2 s on USB, 60 s on battery.
    uint16_t sensorPeriod = g_charging ? SENSOR_PERIOD_S : SENSOR_PERIOD_BATT_S;
    // While the sensor is faulty, probe more often (cap the period to SENSOR_RETRY_S)
    // so that a recovery clears "SENSOR ERROR" within a few seconds instead of up to
    // 60 s on battery. Costs a little power only during an actual fault (rare).
    if (!g_sensorOk && sensorPeriod > SENSOR_RETRY_S) sensorPeriod = SENSOR_RETRY_S;
    bool sensorActive = !g_displayOff || (g_connHandle != BLE_CONN_HANDLE_INVALID);
    if (sensorActive && (g_sec % sensorPeriod == 0)) readSensor();
    if (g_sec % LOG_PERIOD_S == 0) logStatus();
    updateBleData();
    g_needDraw = true;
  }

  // 4) Expire a pending BLE time-set request.
  if (g_pendingValid && (int32_t)(millis() - g_pendingExpiryMs) >= 0) {
    g_pendingValid = false;
    g_needDraw = true;
    LOG_PRINTLN("BLE time-set request expired.");
  }
  if (g_pendingValid && g_displayOff) wakeNow();   // always show the code

  // 5) Power state decision.
  //    - manual sleep (long press)  -> sleep regardless of power source
  //    - automatic sleep            -> only on battery (no USB) and below 50%
  //    - USB present / charging      -> never auto-sleeps
  //    - a pending confirm code      -> keeps the device awake
  bool wantSleep  = (!g_pendingValid) &&
                    ( g_forceSleep ||
                      (!g_charging && g_battPct < SLEEP_BATT_THRESHOLD) );
  bool windowOpen = (int32_t)(millis() - g_wakeUntil) < 0;

  if (wantSleep && !windowOpen) {
    enterSleep();                       // display off (once); BLE handled there
  } else {
    if (g_displayOff) {
      display.ssd1306_command(SSD1306_DISPLAYON);
      g_displayOff = false;
      g_needDraw = true;
    }
    if (g_needDraw) {
      g_needDraw = false;
      if (g_pendingValid) drawConfirmScreen();
      else                drawMainScreen();
    }
  }

  // 6) Low-power wait. The core's FreeRTOS runs in TICKLESS mode, so delay()
  //    actually sleeps the CPU for its whole duration (it does NOT busy-wait,
  //    and waitForEvent() would be worse - it wakes every 1 ms FreeRTOS tick).
  //    Strategy: poll fast only while a button is held; sleep longest when the
  //    screen is off (the big power win); stay responsive to taps when awake
  //    (the OLED dominates power then, so the wake rate barely matters).
  if (digitalRead(BUTTON_PIN) == LOW) delay(BTN_POLL_MS);     // button held -> poll fast
  else if (g_displayOff)              delay(IDLE_SLEEP_MS);   // asleep -> deepest idle
  else                                delay(IDLE_AWAKE_MS);   // awake  -> responsive taps
}
