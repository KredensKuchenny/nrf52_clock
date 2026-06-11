# Dokumentacja — nRF52840 SuperMini: Zegar / Termo‑higrometr BLE

**Plik firmware:** `nrf52_clock.ino` · **Aplikacja towarzysząca:** `nrf52_clock_console.html` (Web Bluetooth)
**Rdzeń płytki:** [pdcook/nRFMicro-Arduino-Core](https://github.com/pdcook/nRFMicro-Arduino-Core) (FreeRTOS + SoftDevice S140)

Ten dokument opisuje architekturę, każdy podsystem, kluczowe fragmenty kodu, przykłady działania oraz przypadki brzegowe (wyjątki). Wszystkie opisy zweryfikowano względem kodu źródłowego.

---

## Spis treści
1. [Przegląd i architektura](#1-przegląd-i-architektura)
2. [Sprzęt i piny](#2-sprzęt-i-piny)
3. [Kompilacja i wgrywanie](#3-kompilacja-i-wgrywanie)
4. [Stałe konfiguracyjne](#4-stałe-konfiguracyjne)
5. [Stan globalny](#5-stan-globalny)
6. [Pomiar czasu — RTC2](#6-pomiar-czasu--rtc2)
7. [Kalendarz i epoch](#7-kalendarz-i-epoch)
8. [Strefa czasowa i DST](#8-strefa-czasowa-i-dst-polska)
9. [Wyświetlacz OLED](#9-wyświetlacz-oled)
10. [Czujnik AHT](#10-czujnik-aht)
11. [Bateria i ochrona ogniwa](#11-bateria-i-ochrona-ogniwa)
12. [Model zasilania i uśpienia](#12-model-zasilania-i-uśpienia)
13. [Przyciski](#13-przyciski)
14. [BLE — usługa i charakterystyki](#14-ble--usługa-i-charakterystyki)
15. [BLE — cykl rozgłaszania](#15-ble--cykl-rozgłaszania)
16. [BLE — strumień danych](#16-ble--strumień-danych)
17. [BLE — ustawianie czasu z potwierdzeniem](#17-ble--ustawianie-czasu-z-potwierdzeniem)
18. [Generator losowy (TRNG)](#18-generator-losowy-trng)
19. [Pętla główna i energia](#19-pętla-główna-i-energia)
20. [Tabela przypadków brzegowych](#20-tabela-przypadków-brzegowych-wyjątki)
21. [Aplikacja webowa](#21-aplikacja-webowa-skrót)
22. [Przykłady z życia](#22-przykłady-z-życia-scenariusze)
23. [Ograniczenia i pomysły](#23-znane-ograniczenia-i-pomysły)

---

## 1. Przegląd i architektura

Urządzenie to zegar z kalendarzem oraz pomiarem temperatury i wilgotności, z interfejsem BLE do odczytu danych na żywo i zdalnego ustawiania czasu. Sercem jest nRF52840; czas utrzymuje sprzętowy licznik RTC2, niezależny od reszty programu.

Podsystemy i ich zależności:

```
            ┌────────────────────────────────────────────┐
            │                  loop()                      │
            │  (FreeRTOS task, tickless idle, delay())     │
            └───┬───────┬──────────┬───────────┬───────────┘
                │       │          │           │
        handleButton  tick     power FSM    BLE update
            │        (1 Hz)        │           │
            ▼          ▼           ▼           ▼
      buttonISR   RTC2 ISR    enterSleep/   dataChar.notify
     (GPIOTE)   (COMPARE0)   System OFF     (SoftDevice S140)
                    │
                    └──> g_epoch++ (UTC), g_tickFlag=true
```

Zasada działania jest **zdarzeniowa**: przerwania (RTC2, przycisk, BLE) ustawiają flagi/stan, a pętla je obsługuje i większość czasu śpi (`delay()` w trybie tickless faktycznie usypia CPU).

---

## 2. Sprzęt i piny

| Element | Interfejs | Adres / pin | Uwagi |
|---|---|---|---|
| AHT10/20 (temp/wilgotność) | I2C | `0x38` | `Adafruit_AHTX0` |
| SSD1306 OLED 128×32 | I2C | `0x3C` | `Adafruit_SSD1306` |
| Przycisk | GPIO | `D8`, aktywny **LOW** | `INPUT_PULLUP` |
| Bateria LiPo | wewn. ADC | VDDH/5 | `analogReadVDDHDIV5()` |

> **Wyjątek sprzętowy:** płytka SuperMini **nie ma** układu ochrony niskonapięciowej — układ ładujący tylko ładuje. Dlatego ochronę ogniwa realizuje firmware (sekcja 11).

---

## 3. Kompilacja i wgrywanie

- Arduino IDE → **Tools → Board → „SuperMini nRF52840"** (rdzeń pdcook/nRFMicro).
- Biblioteki: `Adafruit_GFX`, `Adafruit_SSD1306`, `Adafruit_AHTX0`, `Bluefruit (bluefruit.h)`.
- Rdzeń uruchamia **FreeRTOS w trybie tickless** i **SoftDevice S140** (BLE). SoftDevice startuje przez `Bluefruit.begin()`, co jednocześnie uruchamia LFCLK potrzebny dla RTC2.

---

## 4. Stałe konfiguracyjne

Najważniejsze `#define` (sekcja CONFIGURATION):

```cpp
#define BUTTON_PIN          8
#define BTN_DEBOUNCE_MS     30
#define BTN_LONGPRESS_MS    1500

#define BTN_POLL_MS         15    // odpytywanie gdy przycisk trzymany
#define IDLE_AWAKE_MS       100   // pętla przy włączonym ekranie
#define IDLE_SLEEP_MS       200   // pętla przy zgaszonym ekranie

#define BATT_CAL            0.9831f  // kalibracja: 4.060 V (DMM) / 4.13 V (odczyt)
#define BATT_FULL_V         4.20f   // 100%
#define BATT_EMPTY_V        3.00f   // 0%
#define BATT_CHARGING_V     4.25f   // zapasowe wykrycie ładowania
#define BATT_PROTECT_V      3.00f   // System OFF poniżej
#define BATT_PROTECT_COUNT  3       // kolejne niskie odczyty (×5 s)
#define BATT_OVERSAMPLE     16
#define BATT_EMA_ALPHA      0.25f

#define SLEEP_BATT_THRESHOLD 50              // [%] poniżej -> sen
#define WAKE_WINDOW_MS       (5UL*60*1000)   // 5 min czuwania po wybudzeniu
#define ADV_BATT_WINDOW_MS   (30UL*60*1000)  // 30 min rozgłaszania na baterii

#define SENSOR_PERIOD_S       2   // odczyt AHT na USB
#define SENSOR_PERIOD_BATT_S  60  // odczyt AHT na baterii
#define BATTERY_PERIOD_S      5
#define LOG_PERIOD_S          10

#define OLED_CONTRAST_DAY    0x80
#define OLED_CONTRAST_NIGHT  0x00
#define NIGHT_START_H        20   // noc 20:00..08:00 (czas lokalny)
#define NIGHT_END_H          8

#define TXPOWER_USB          4    // dBm
#define TXPOWER_BATT        -4    // dBm
#define CONFIRM_TIMEOUT_MS   60000UL

#define ENABLE_SERIAL_LOG    0    // 0 = bez Serial (mniejszy pobór); 1 = logi debug @115200
```

> **`ENABLE_SERIAL_LOG` (przełącznik logowania):** przy `0` Serial (USB‑CDC) **nie jest w ogóle uruchamiany**, nie buduje się też stringów logów — mniej prądu i CPU; to domyślne ustawienie produkcyjne. Wszystkie wywołania idą przez makra `LOG_PRINT/LOG_PRINTLN/LOG_BEGIN/LOG_FLUSH`, które przy `0` rozwijają się do `((void)0)`. Ustaw `1` tylko na czas debugowania.

> **Dlaczego `BATT_CAL=0.9831`:** ADC pokazywał 4.13 V przy realnych 4.060 V (multimetr). Współczynnik = 4.060/4.13. Jeśli kalibrujesz własną płytkę, zmierz miernikiem i podstaw `V_real / V_shown`.

---

## 5. Stan globalny

```cpp
volatile uint32_t g_epoch    = 0;     // sekundy od 2000-01-01 00:00:00 UTC
volatile bool     g_tickFlag = false; // ustawia ISR RTC2 raz/s
uint32_t          g_sec      = 0;     // licznik do harmonogramu zadań

float g_tempC, g_hum, g_vddh, g_vddhFilt;   // pomiary + filtr napięcia
int   g_battPct;  bool g_charging;           // procent + ładowanie
uint8_t g_lowVoltCount;                       // licznik niskich odczytów

bool     g_displayOff;    // ekran zgaszony (sen)
bool     g_forceSleep;    // jednorazowe żądanie snu (długi przycisk)
uint32_t g_wakeUntil;     // koniec 5-min okna czuwania
uint8_t  g_curContrast;   // aktualny kontrast OLED (anty-spam I2C)

bool     g_bleOn;         // intencja rozgłaszania (też "BT" na OLED)
uint32_t g_advUntil;      // koniec 30-min okna rozgłaszania (bateria)
uint16_t g_connHandle;    // uchwyt połączenia lub BLE_CONN_HANDLE_INVALID
int8_t   g_rssi;          // ostatni RSSI łącza

volatile bool g_pendingValid; uint32_t g_pendingEpoch;   // oczekujące ustawienie czasu
uint16_t g_pendingCode; uint32_t g_pendingExpiryMs;

volatile bool g_buttonIrq;  // ustawia ISR przycisku (każde wciśnięcie)
volatile bool g_justWoke;   // wciśnięcie wybudziło urządzenie
```

> **Wyjątek (współbieżność):** `g_epoch` jest modyfikowany w przerwaniu RTC2, więc każdy odczyt 32‑bitowej wartości jest atomowy:
> ```cpp
> __disable_irq(); e = g_epoch; __enable_irq();
> ```
> To chroni przed odczytaniem „w połowie" inkrementacji.

---

## 6. Pomiar czasu — RTC2

PRESCALER RTC jest **12‑bitowy** (max 4095), więc najwolniejszy licznik to `32768/(4095+1) = 8 Hz`. Zegar 1 Hz bezpośrednio jest **niemożliwy**. Rozwiązanie: licznik biegnie 8 Hz, a zdarzenie **COMPARE** wypada co 8 zliczeń → jedno przerwanie na sekundę.

```cpp
extern "C" void RTC2_IRQHandler(void) {
  if (NRF_RTC2->EVENTS_COMPARE[0]) {
    NRF_RTC2->EVENTS_COMPARE[0] = 0;
    (void)NRF_RTC2->EVENTS_COMPARE[0];                  // flush (errata)
    NRF_RTC2->CC[0] = (NRF_RTC2->CC[0] + 8) & 0xFFFFFF; // następna sekunda
    g_epoch++;
    g_tickFlag = true;
  }
}
```

```cpp
void setupRTC2(void) {
  NRF_RTC2->TASKS_STOP=1; NRF_RTC2->TASKS_CLEAR=1;
  NRF_RTC2->PRESCALER = 4095;                       // 8 Hz
  NRF_RTC2->CC[0]     = 8;                           // pierwsze dopasowanie = 1 s
  NRF_RTC2->EVTENSET  = RTC_EVTENSET_COMPARE0_Msk;
  NRF_RTC2->INTENSET  = RTC_INTENSET_COMPARE0_Msk;
  NVIC_SetPriority(RTC2_IRQn, 7);                   // najniższy prio (zgodny z SoftDevice)
  NVIC_EnableIRQ(RTC2_IRQn);
  NRF_RTC2->TASKS_START = 1;
}
```

**Dlaczego maska `& 0xFFFFFF`:** licznik RTC jest 24‑bitowy. `CC += 8` z maską poprawnie „zawija się" przy przepełnieniu (2²⁴ dzieli się przez 8, więc wyrównanie sekund jest zachowane). Czas nie jest liczony z `COUNTER`, tylko przez zliczanie przerwań — więc przepełnienie licznika (co ~24 dni) nie wpływa na zegar.

> **Wybór RTC2:** RTC0 należy do SoftDevice, RTC1 do FreeRTOS. RTC2 jest wolny dla aplikacji.
> **Wyjątek (dokładność):** dokładność zależy od źródła LFCLK (kwarc 32768 Hz vs RC). Na SuperMini jest kwarc, którego używa SoftDevice — dryf jest mały. Metoda podziału nie wprowadza błędu (8 Hz × 8 = dokładnie 1 Hz).

---

## 7. Kalendarz i epoch

`g_epoch` to liczba sekund od **2000‑01‑01 00:00:00** (to była **sobota** → wykorzystywane do obliczania dnia tygodnia).

```cpp
uint32_t toEpoch(uint16_t year, uint8_t month, uint8_t day, uint8_t hh, uint8_t mm, uint8_t ss) {
  uint32_t days = 0;
  for (uint16_t y = 2000; y < year; y++)  days += isLeapYear(y) ? 366 : 365;
  for (uint8_t  m = 1;    m < month; m++) days += daysInMonth(year, m);
  days += (uint32_t)(day - 1);
  return (((days*24UL)+hh)*60UL+mm)*60UL+ss;
}
```

`fromEpoch()` odwraca to i wylicza dzień tygodnia:
```cpp
wday = (uint8_t)((days + 6) % 7);   // 0=Sun..6=Sat (bo dzień 0 = sobota)
```

Lata przestępne: `((y%4==0 && y%100!=0) || y%400==0)`. Długości miesięcy z tablicy + luty 29 w roku przestępnym.

**Przykład:** `toEpoch(2026,6,7,14,30,0)` → liczba sekund; `fromEpoch()` zwróci z powrotem `2026-06-07 14:30:00`, `wday=0` (niedziela).

---

## 8. Strefa czasowa i DST (Polska)

**Kluczowa decyzja projektowa:** `g_epoch` przechowuje **UTC**, a czas lokalny liczony jest „w locie". Dzięki temu DST jest deterministyczną funkcją chwili — **brak dwuznacznej/powtórzonej godziny** przy zmianie jesiennej i transmisje są poprawne, nawet jeśli urządzenie spało w momencie przełączenia.

Reguła: **CEST (UTC+2)** od ostatniej niedzieli marca 01:00 UTC do ostatniej niedzieli października 01:00 UTC; inaczej **CET (UTC+1)**.

```cpp
int plOffsetH(uint32_t utc) {            // 1=CET, 2=CEST
  uint16_t Y; uint8_t mo,d,hh,mm,ss,wd; fromEpoch(utc, Y,mo,d,hh,mm,ss,wd);
  if (mo < 3 || mo > 10) return 1;
  if (mo > 3 && mo < 10) return 2;
  uint8_t ls = lastSundayDay(Y, mo);
  if (mo == 3) return (d > ls || (d==ls && hh>=1)) ? 2 : 1;  // wiosna 01:00 UTC
  return               (d < ls || (d==ls && hh< 1)) ? 2 : 1;  // jesień 01:00 UTC
}
uint32_t plLocal(uint32_t utc) { return utc + (uint32_t)plOffsetH(utc)*3600UL; }
```

Wszystkie wyjścia (OLED, BLE, logi) używają `fromEpoch(plLocal(g_epoch), ...)`.

Przy **ustawianiu** czasu użytkownik podaje czas lokalny, więc konwertujemy lokalny→UTC, używając granic w czasie lokalnym (wiosna 02:00, jesień 03:00):
```cpp
g_pendingEpoch = toEpoch(Y,mo,d,hh,mm,ss) - plOffsetFromLocalH(Y,mo,d,hh)*3600UL;
```

**Przykład:** ustawiasz `07-06-2026 14:30:00` (lato) → zapis UTC `12:30:00`; OLED pokaże `14:30:00`.
**Przykład DST:** ostatnia niedziela października, 03:00 lokalnego → zegar pokaże ponownie 02:00 (cofnięcie); brak pętli, bo decyduje chwila UTC.

> **Wyjątek (godzina nieistniejąca/podwójna przy ręcznym ustawianiu):** jeśli ustawisz czas dokładnie w „dziurze" wiosennej lub w powtórzonej godzinie jesiennej, offset lokalny→UTC może być o 1 h przesunięty. To akceptowalny przypadek brzegowy — normalnie nie ustawia się czasu w tej jednej godzinie.

---

## 9. Wyświetlacz OLED

Układ (128×32):
```
[HH:MM](rozmiar 2)  :SS            87%     <- linia 1: bateria % lub "CHG"
                                   3.85V    <- linia 2: napięcie (tylko na baterii)
Pon 06-06-2026                              <- dzień tygodnia + DD-MM-YYYY
T 23.5C  H 45%                        BT*   <- temp/wilgotność + stan BLE
```

**Nocne ściemnianie** (w `drawMainScreen`, na czasie lokalnym):
```cpp
uint8_t wantC = (hh >= NIGHT_START_H || hh < NIGHT_END_H) ? OLED_CONTRAST_NIGHT : OLED_CONTRAST_DAY;
if (wantC != g_curContrast) {                 // wysyłaj I2C tylko przy zmianie
  g_curContrast = wantC;
  display.ssd1306_command(SSD1306_SETCONTRAST);
  display.ssd1306_command(wantC);
}
```

**Wskaźnik BLE** kieruje się rzeczywistym połączeniem, nie intencją rozgłaszania:
```cpp
if      (Bluefruit.connected()) printRightAligned("BT*", 24, 1);  // połączony
else if (g_bleOn)               printRightAligned("BT",  24, 1);  // rozgłasza
// brak -> BLE wyłączone
```

> **Dlaczego `Bluefruit.connected()` a nie `g_bleOn`:** połączenie utrzymane przez sen ma `g_bleOn=false` (rozgłaszanie zatrzymane), ale klient nadal jest podłączony — `BT*` musi się świecić.
> **Przykład:** o 22:00 ekran sam przygasa (kontrast `0x40`), o 06:00 wraca do `0xCF`.

---

## 10. Czujnik AHT

```cpp
void readSensor(void) {
  sensors_event_t humidity, temp;
  // Healthy only if the read succeeds AND values are physically plausible.
  if (aht.getEvent(&humidity, &temp) &&
      humidity.relative_humidity >= 0 && humidity.relative_humidity <= 100 &&
      temp.temperature > -40 && temp.temperature < 85) {
    g_tempC = temp.temperature; g_hum = humidity.relative_humidity;
    g_sensorOk = true;
  } else {
    g_sensorOk = false;   // missing/garbled AHT
  }
}
```

Stan zdrowia czujnika jest też ustawiany przy starcie (`g_sensorOk = aht.begin()`), a na ekranie zamiast zer pokazuje się wyraźny błąd:
```cpp
if (g_sensorOk) snprintf(hbuf, sizeof(hbuf), "T %.1fC  H %.0f%%", g_tempC, g_hum);
else            snprintf(hbuf, sizeof(hbuf), "SENSOR ERROR");   // awaria AHT na OLED
```

Harmonogram (w pętli 1 Hz):
```cpp
uint16_t sensorPeriod = g_charging ? SENSOR_PERIOD_S : SENSOR_PERIOD_BATT_S;  // 2 s / 60 s
bool sensorActive = !g_displayOff || (g_connHandle != BLE_CONN_HANDLE_INVALID);
if (sensorActive && (g_sec % sensorPeriod == 0)) readSensor();
```

> **Sygnalizacja awarii:** błąd czujnika jest teraz widoczny dla użytkownika w **trzech** miejscach: na OLED (`SENSOR ERROR`), w strumieniu BLE (pole `S=0`, sekcja 16) i na stronie WWW (czerwone `BŁĄD` w kafelkach temp/wilgotność). Wartości z błędnego odczytu **nie trafiają na wykresy**.
> **Dynamicznie w czasie pracy:** `g_sensorOk` jest aktualizowane przy **każdym** odczycie, więc awaria w trakcie działania pojawia się sama, a po naprawie ekran **wraca** do pokazywania temperatury/wilgotności — bez restartu. Wykrycie awarii następuje przy najbliższym odczycie (do 2 s na USB, do 60 s na baterii — tyle samo, co normalne odświeżanie danych). Gdy czujnik jest w błędzie, sprawdzamy go częściej (`SENSOR_RETRY_S = 5 s`), żeby **powrót do działania pokazał się w ~5 s**, a nie dopiero po 60 s; szybsze odpytywanie kosztuje trochę prądu tylko podczas realnej awarii.
> **Czytanie w śnie tylko gdy połączony:** przy zgaszonym ekranie czujnik jest odczytywany **wyłącznie, gdy ktoś jest podłączony przez BLE** (logowanie na żywo). Bez połączenia w śnie — zero odczytów (najmniejszy pobór). Częstotliwość dalej zależy od zasilania: **2 s na USB**, **60 s na baterii** (również w śnie).
> **Plausibility check:** uznajemy odczyt za poprawny tylko, gdy wilgotność mieści się w 0–100 % i temperatura w −40…85 °C — to wyłapuje też „zamrożone śmieci" z odłączonego czujnika, nie tylko twardy błąd I2C.

---

## 11. Bateria i ochrona ogniwa

**Wykrywanie zasilania zewnętrznego** — niezawodnie po komparatorze USB VBUS:
```cpp
bool usbConnected(void) {
  return (NRF_POWER->USBREGSTATUS & POWER_USBREGSTATUS_VBUSDETECT_Msk) != 0;
}
```

**Odczyt napięcia** — oversampling + filtr EMA + reseed przy zmianie USB:
```cpp
for (int i=0;i<BATT_OVERSAMPLE;i++) acc += analogReadVDDHDIV5();
float v = (acc/BATT_OVERSAMPLE) * (ADC_REF/ADC_MAX) * VDDH_DIVIDER * BATT_CAL;
if (!g_battInit || usb != usbPrev) { g_vddhFilt = v; ... }   // snap przy plug/unplug
else                                 g_vddhFilt += BATT_EMA_ALPHA*(v - g_vddhFilt);
g_charging = usb || (g_vddh > BATT_CHARGING_V);
```

**Procent** (liniowo 3.00–4.20 V) z **histerezą** ±2% (anty‑migotanie):
```cpp
int p = batteryPercent(g_vddh);
if (g_charging || g_battPct==0 || abs(p - g_battPct) >= 2) g_battPct = p;
```

**Ochrona przed głębokim rozładowaniem** (urządzenie samo, bo płytka nie ma cut‑offu):
```cpp
void checkBatteryProtection(void) {
  if (!g_battInit) return;
  if (!g_charging && g_vddh <= BATT_PROTECT_V) {
    if (++g_lowVoltCount >= BATT_PROTECT_COUNT) enterSystemOff();  // 3 kolejne odczyty
  } else g_lowVoltCount = 0;
}
```

`enterSystemOff()` pokazuje komunikat, zatrzymuje BLE, ustawia przycisk jako źródło wybudzenia (SENSE=LOW) i wchodzi w System OFF (`sd_power_system_off()`).

> **Wyjątek (utrata czasu):** wybudzenie z System OFF **resetuje** MCU — RAM i zegar są tracone, czas wraca do domyślnego i trzeba go ustawić ponownie przez BLE.
> **Wyjątek (chwilowy spadek):** wymóg 3 kolejnych niskich odczytów (~15 s) chroni przed wyłączeniem na skutek krótkiego spadku napięcia (np. szpilka przy nadawaniu radia).
> **Wyjątek (ładowanie):** podczas ładowania `%` jest niemiarodajny → pokazywany `CHG`, napięcie ukryte; ochrona nie działa (bo nie na baterii).

---

## 12. Model zasilania i uśpienia

Decyzja o śnie w pętli:
```cpp
bool wantSleep  = (!g_pendingValid) &&
                  ( g_forceSleep || (!g_charging && g_battPct < SLEEP_BATT_THRESHOLD) );
bool windowOpen = (int32_t)(millis() - g_wakeUntil) < 0;   // 5-min okno czuwania
if (wantSleep && !windowOpen) enterSleep();
```

`enterSleep()` gasi ekran (raz) i zatrzymuje rozgłaszanie, **nie zrywając** aktywnego połączenia:
```cpp
void enterSleep(void) {
  if (!g_displayOff) {
    display.ssd1306_command(SSD1306_DISPLAYOFF);
    g_displayOff = true;
    advStopKeepConn();   // stop reklamy; połączenie żyje do rozłączenia
  }
}
```

Trzy stany:

| Stan | Auto-sen | Czujnik | TX | Rozgłaszanie |
|---|---|---|---|---|
| **USB** | nie | 2 s | +4 dBm | bez limitu |
| **Bateria ≥50%** | nie | 60 s | −4 dBm | max 30 min (idle) |
| **Bateria <50%** | tak (okno 5 min) | 60 s | −4 dBm | tylko gdy włączysz w oknie |

**Przykłady:**
- *USB:* wpięty rano, wieczorem dalej świeci; nigdy nie zasypia.
- *Bateria 70%:* czuwa; po 30 min bez połączenia rozgłaszanie gaśnie, ekran dalej świeci.
- *Bateria 40%:* ekran zgaszony; klik budzi na 5 min; brak akcji → znów gaśnie.

> **Wyjątek (`g_pendingValid`):** gdy na ekranie jest kod potwierdzenia, `wantSleep=false` — urządzenie nie zaśnie, dopóki kod nie wygaśnie/zostanie użyty.

---

## 13. Przyciski

ISR tylko „zatrzaskuje" zdarzenie i wybudza CPU:
```cpp
void buttonISR(void) { g_buttonIrq = true; }   // falling edge (wciśnięcie)
```

Wybudzenie kasuje żądanie snu i otwiera 5‑min okno:
```cpp
void wakeNow(void) {
  g_forceSleep = false;
  g_wakeUntil = millis() + WAKE_WINDOW_MS;
  if (g_displayOff) { display ON; g_displayOff=false; g_needDraw=true; }
}
```

Długie przyciśnięcie = **jednorazowa** komenda „uśpij teraz" (nie toggle):
```cpp
void doLongPress(void) { g_forceSleep = true; g_wakeUntil = millis(); }  // zamyka okno -> sen
```

Odpytywanie z debounce i wykrywaniem długiego przyciśnięcia (`handleButton`): krótkie puszczenie (gdy to nie był „przycisk budzący" ani nie odpalił long‑press) → toggle rozgłaszania:
```cpp
if (!wakePress && !longFired) { if (g_bleOn) bleStop(); else bleStart(); }
```
Long‑press odpala dokładnie po 1.5 s trzymania:
```cpp
if (prevStable==LOW && !wakePress && !longFired && (ms - tPressStart) >= BTN_LONGPRESS_MS) {
  longFired = true; doLongPress();
}
```

**Przykłady i wyjątki:**
- Ekran świeci, klik → `BT` (rozgłasza); klik → wyłączone.
- Śpi, klik → ekran wstaje (`wakePress=true`, **nie** toggluje BLE); kolejny klik → włącza rozgłaszanie.
- Trzymanie ≥1.5 s → ekran gaśnie natychmiast.
- *Wyjątek:* trzymanie przycisku **budzącego** nie wyzwala long‑press (`wakePress` blokuje), więc nie zaśniesz przypadkiem zaraz po wybudzeniu.
- *Wyjątek:* przy `IDLE_AWAKE_MS=100` tapnięcia krótsze niż ~100 ms mogą się nie zarejestrować; normalne kliknięcia są dłuższe.

---

## 14. BLE — usługa i charakterystyki

Usługa 128‑bit, baza `0F1E2D3C-4B5A-6978-8796-A5B4C3D2E1xx`:

| Charakterystyka | Sufiks | Właściwości | maxLen | Funkcja |
|---|---|---|---|---|
| Service | `E100` | — | — | usługa zegara |
| DATA | `E101` | READ + NOTIFY | 80 | strumień danych |
| SETTIME | `E102` | WRITE (+WO_RESP) | 48 | żądanie ustawienia czasu |
| CONFIRM | `E103` | WRITE (+WO_RESP) | 16 | kod potwierdzenia |

Nazwa `nRF52-Clock` jest w **scan response** (128‑bit UUID wypełnia pakiet rozgłoszeniowy). Producent `_KrEdEnS_`, model `SuperMini Clock` (Device Information Service).

**MTU 247** zamiast domyślnego 23 — kluczowe, bo notyfikacja mieści tylko `MTU−3` bajtów, a string danych ma ~60 B:
```cpp
Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);   // PRZED Bluefruit.begin()
```
Parametry reklamy: interwał `32..244` (×0.625 ms = 20..152.5 ms), fast timeout 30 s.

> **Wyjątek (bezpieczeństwo):** uprawnienia to `SECMODE_OPEN` — brak parowania/szyfrowania. Kod potwierdzenia chroni przed przypadkową zmianą czasu, ale nie przed celowym intruzem. Odczyt danych jest otwarty.

---

## 15. BLE — cykl rozgłaszania

Funkcje sterujące:
```cpp
void bleStart(void){ Advertising.restartOnDisconnect(true); Advertising.start(0);
                     g_bleOn=true; g_advUntil=millis()+ADV_BATT_WINDOW_MS; }   // arm 30 min
void bleStop(void){ Advertising.restartOnDisconnect(false); Advertising.stop();
                    if (connected) disconnect(); g_bleOn=false; }              // ręczne OFF (+rozłącz)
void advStopKeepConn(void){ Advertising.restartOnDisconnect(false); Advertising.stop();
                            g_bleOn=false; }                                   // sen: nie zrywaj połączenia
```

Limit 30 min (na baterii, gdy nikt niepodłączony) — w pętli:
```cpp
if (!g_displayOff && g_bleOn && !g_charging &&
    g_connHandle==BLE_CONN_HANDLE_INVALID && (int32_t)(millis()-g_advUntil)>=0) {
  Advertising.stop(); g_bleOn=false;   // oszczędzanie
}
```

Reset po rozłączeniu (gdy czuwa i BLE chciane):
```cpp
void disconnectCb(...) {
  g_connHandle = BLE_CONN_HANDLE_INVALID;
  if (!g_displayOff && g_bleOn) g_advUntil = millis() + ADV_BATT_WINDOW_MS;  // świeże 30 min
  else { Advertising.stop(); g_bleOn=false; }                               // sen/wyłączone -> zostaw off
}
```

**Przykłady:**
- *USB:* `g_charging` blokuje warunek limitu → rozgłasza bez końca.
- *Reset 30 min:* 10:00 start; 10:10 połączenie; 10:13 rozłączenie → nowe 30 min do 10:43.
- *Sen z połączeniem:* long‑press przy połączonym kliencie → ekran gaśnie, `advStopKeepConn()` zatrzymuje reklamę, ale połączenie żyje; po rozłączeniu `disconnectCb` zostawia BLE off.

---

## 16. BLE — strumień danych

`updateBleData()` co sekundę składa string i wysyła notyfikację:
```cpp
fromEpoch(plLocal(g_epoch), ...);                  // czas lokalny
if (g_connHandle != BLE_CONN_HANDLE_INVALID) {     // RSSI tylko gdy połączony
  int8_t r; uint8_t ch; if (sd_ble_gap_rssi_get(g_connHandle,&r,&ch)==NRF_SUCCESS) g_rssi=r;
}
snprintf(buf, 80, "%02u-%02u-%04u %02u:%02u:%02u;T=%.1f;H=%.1f;B=%d;V=%.2f;CHG=%d;R=%d;S=%d",
         d,mo,Y, hh,mm,ss, g_tempC,g_hum,g_battPct,g_vddh, g_charging?1:0, g_rssi, g_sensorOk?1:0);
dataChar.write(buf,n);
if (Bluefruit.connected()) dataChar.notify(buf,n);
```

**Format pól:**

| Pole | Znaczenie | Przykład |
|---|---|---|
| data/godz. | `DD-MM-YYYY HH:MM:SS` (lokalny) | `07-06-2026 14:32:07` |
| `T` | temperatura °C | `T=23.5` |
| `H` | wilgotność % | `H=45.2` |
| `B` | bateria % | `B=87` |
| `V` | napięcie V | `V=3.85` |
| `CHG` | ładowanie 0/1 | `CHG=0` |
| `R` | RSSI dBm | `R=-58` |
| `S` | czujnik OK 1 / awaria 0 | `S=1` |

Pełny przykład: `07-06-2026 14:32:07;T=23.5;H=45.2;B=87;V=3.85;CHG=0;R=-58;S=1` (65 B < bufor 80 B).

> **Wyjątek:** RSSI mierzy firmware (Web Bluetooth nie udostępnia RSSI połączonego urządzenia). RSSI ważne tylko gdy połączony; przy braku połączenia pole zawiera ostatnią wartość, ale notyfikacje i tak idą tylko do połączonego klienta.

---

## 17. BLE — ustawianie czasu z potwierdzeniem

Trójkrokowy, odporny na przypadkową zmianę:

1. Klient zapisuje do **SETTIME**: `DD-MM-YYYY HH:MM:SS` (czas lokalny). Firmware waliduje i przelicza na UTC:
```cpp
if (sscanf(buf,"%d-%d-%d %d:%d:%d",&d,&mo,&Y,&hh,&mm,&ss)==6 && /* zakresy */ ) {
  g_pendingEpoch = toEpoch(Y,mo,d,hh,mm,ss) - plOffsetFromLocalH(Y,mo,d,hh)*3600UL;
  g_pendingCode  = random4digit();                  // sprzętowo losowy
  g_pendingExpiryMs = millis() + CONFIRM_TIMEOUT_MS; // 60 s
  g_pendingValid = true;
}
```
2. OLED pokazuje **4‑cyfrowy kod**, ważny **60 s**.
3. Klient zapisuje kod do **CONFIRM**:
```cpp
if (!g_pendingValid) return;
if ((int32_t)(millis()-g_pendingExpiryMs) >= 0) { g_pendingValid=false; return; }  // wygasło
if (code == g_pendingCode) {
  __disable_irq();
  g_epoch = g_pendingEpoch;
  NRF_RTC2->TASKS_CLEAR=1; NRF_RTC2->CC[0]=8;   // wyrównaj sekundę: start od zera
  __enable_irq();
  g_pendingValid=false;
}
```

**Przykład (ręczny):** wyślij `07-06-2026 14:31:00` → ekran `2187` → wyślij `2187` → zegar ustawiony, sekundy ruszają od 0.
**Przykład (strona, „na styk"):** „Pełna minuta ↑" wstawia najbliższą pełną minutę; „Auto o godzinie" sam wysyła kod dokładnie gdy zegar komputera ją wybije → urządzenie startuje równo na :00.

> **Wyjątki:** zły kod → `Wrong confirm code` (bez zmiany); po 60 s → `Request expired`; format niezgodny lub zła data (np. 31‑02) → odrzucone w walidacji.

---

## 18. Generator losowy (TRNG)

Kod potwierdzenia musi być nieprzewidywalny, więc pochodzi ze **sprzętowego** generatora (przez SoftDevice, bo to on jest właścicielem peryferium RNG przy włączonym BLE):
```cpp
uint16_t random4digit(void) {
  uint8_t buf[2], avail=0;
  for (int i=0;i<50;i++){ sd_rand_application_bytes_available_get(&avail);
                          if (avail>=2) break; delay(1); }
  if (avail>=2 && sd_rand_application_vector_get(buf,2)==NRF_SUCCESS) {
    uint16_t v = ((uint16_t)buf[0]<<8)|buf[1];  // 0..65535
    return 1000 + (v % 9000);                   // -> 1000..9999
  }
  // Fallback: PRNG zasiany szumem analogowym + timerami (rzadko)
  randomSeed(((uint32_t)analogRead(A0)<<16) ^ micros() ^ analogReadVDDHDIV5());
  return (uint16_t)random(1000,10000);
}
```

> **Wyjątek:** jeśli pula entropii jest pusta i wywołanie SoftDevice zawiedzie, używany jest PRNG (słabszy, ale wystarczający awaryjnie). `v % 9000` daje znikome obciążenie statystyczne — bez znaczenia dla kodu.

---

## 19. Pętla główna i energia

Rdzeń działa w **tickless FreeRTOS**, więc `delay()` faktycznie usypia CPU (nie kręci się w pętli). `waitForEvent()` byłby gorszy (budzi co 1 ms tik FreeRTOS), dlatego stosujemy `delay()` o adaptacyjnej długości:

```cpp
void loop(void) {
  if (g_buttonIrq) { g_buttonIrq=false; if (g_displayOff){ wakeNow(); g_justWoke=true; } }  // 1) wybudzenie
  handleButton();                                                                            // 2) przyciski
  /* 2b) reakcja na USB plug/unplug: readBattery + setTxPower + reset 30 min  */
  /* 2c) limit 30 min rozgłaszania (bateria, idle)                            */
  if (g_tickFlag) {                                                                          // 3) zadania 1 Hz
    g_tickFlag=false; g_sec++;
    if (g_sec%BATTERY_PERIOD_S==0){ readBattery(); checkBatteryProtection(); }
    if (sensorActive && g_sec%sensorPeriod==0) readSensor();  // też w śnie, gdy połączony
    if (g_sec%LOG_PERIOD_S==0) logStatus();
    updateBleData(); g_needDraw=true;
  }
  /* 4) wygaśnięcie kodu potwierdzenia                                        */
  /* 5) decyzja o śnie / rysowanie ekranu                                     */
  // 6) sen niskoenergetyczny:
  if (digitalRead(BUTTON_PIN)==LOW) delay(BTN_POLL_MS);    // trzymany -> szybkie odpytywanie
  else if (g_displayOff)            delay(IDLE_SLEEP_MS);  // śpi -> najgłębszy idle
  else                              delay(IDLE_AWAKE_MS);  // czuwa
}
```

**Przykład:** przy zgaszonym ekranie CPU śpi i budzi się ~5×/s (plus tyknięcie RTC) na mikrosekundy; przy włączonym ~10×/s. Sekundy odświeżają się płynnie, bo tyknięcie jest obsługiwane w ≤100 ms.

> **Wyjątek (opóźnienie wybudzenia):** gdy śpi, pętla czeka do `IDLE_SLEEP_MS=200 ms`, więc reakcja na klik (zapalenie ekranu) ma do ~200 ms opóźnienia.

---

## 20. Tabela przypadków brzegowych (wyjątki)

| Sytuacja | Zachowanie |
|---|---|
| Odczyt `g_epoch` w trakcie inkrementacji ISR | chroniony `__disable_irq()/__enable_irq()` |
| Przepełnienie 24‑bit licznika RTC (~24 dni) | bez wpływu — czas liczony przez przerwania, `CC` z maską |
| Notyfikacja > 20 B przy MTU 23 | rozwiązane: MTU 247 (`configPrphBandwidth`) |
| Ładowanie: `%` niemiarodajny | pokazywany `CHG`, napięcie ukryte; wykres baterii pomijany |
| Krótki dip napięcia (szpilka radia) | ochrona dopiero po 3 kolejnych niskich odczytach (~15 s) |
| Głębokie rozładowanie → System OFF | czas tracony; po wybudzeniu reset i ustawienie od nowa |
| Zmiana czasu w „dziurze" DST | offset lokalny→UTC może być ±1 h (rzadki przypadek) |
| Trzymanie przycisku budzącego | nie wyzwala long‑press (`wakePress`) |
| Tapnięcie < ~100 ms (ekran on) | może się nie zarejestrować (`IDLE_AWAKE_MS=100`) |
| Zły / spóźniony kod potwierdzenia | odrzucony / wygasły po 60 s, bez zmiany zegara |
| Sen przy aktywnym połączeniu | reklama off, połączenie żyje do rozłączenia |
| Awaria / brak czujnika AHT | `g_sensorOk=0`; OLED: `SENSOR ERROR`, BLE: `S=0`, WWW: czerwone `BŁĄD`; błędne wartości nie idą na wykres |
| Brak SoftDevice entropy | fallback PRNG dla kodu |
| Brak `g_ADigitalPinMap`/`A0` w rdzeniu | wymaga dostosowania (zależne od płytki) |

---

## 21. Aplikacja webowa (skrót)

`nrf52_clock_console.html` (Chrome/Edge, Web Bluetooth): łączy się po UUID usługi, pokazuje dane na żywo, ustawia czas (z auto‑potwierdzeniem na pełnej minucie), rysuje **wykresy na żywo** (T/H/bateria/RSSI) z przełącznikiem zakresu 5/10/15/30/60 min i tooltipem (wartość + godzina próbki). Parsuje string z sekcji 16; datę czyta jako `DD-MM-YYYY`; napięcie ukrywa przy ładowaniu. Gdy `S=0` (awaria czujnika) w kafelkach temperatury i wilgotności pokazuje czerwone **`BŁĄD`** i nie dorzuca błędnych próbek do wykresów.

---

## 22. Przykłady z życia (scenariusze)

Konkretne przebiegi „co robisz → co robi urządzenie/ekran/BLE". Wartości (godziny, %, napięcia) są przykładowe.

### 22.1. Na biurku pod USB
1. Wpinasz kabel USB o **09:00**.
2. `usbConnected()` = true → `g_charging=1`, TX rośnie do **+4 dBm**, ekran świeci.
3. OLED: w prawym górnym rogu **`CHG`** (zamiast `%`), napięcie ukryte; czujnik czyta co **2 s**.
4. Rozgłaszanie BLE działa **bez limitu** — telefon/komputer może się połączyć o dowolnej porze.
5. O **18:00** wszystko nadal działa, urządzenie ani razu nie zasnęło.

> Wynik: stacjonarny tryb „zawsze gotowy". Idealny do podglądu na żywo i ustawiania czasu.

### 22.2. Na baterii, normalne użycie (≥50%)
1. Odpinasz USB przy **78%**. `g_charging=0`, TX spada do **−4 dBm**, licznik 30 min rozgłaszania rusza.
2. OLED: `78%` i napięcie np. `3.95V`; czujnik czyta co **60 s**.
3. Urządzenie **nie usypia** (≥50%), ekran świeci, zegar tyka.
4. Nie łączysz się przez 30 min → o **+30 min** rozgłaszanie samo gaśnie (znika `BT`), ekran dalej świeci.
5. Klikasz krótko → znów `BT` (rozgłasza), licznik 30 min od nowa.

> Wynik: oszczędzanie baterii bez usypiania; sam decydujesz klikiem, kiedy rozgłaszać.

### 22.3. Niska bateria (<50%)
1. Bateria spada do **42%**. Po zamknięciu 5‑min okna urządzenie wchodzi w sen: **ekran gaśnie**, zegar chodzi dalej na RTC2.
2. Podchodzisz, **klikasz raz** → ekran wstaje (BLE nadal off), masz **5 min** czuwania.
3. Nie robisz nic przez 5 min → ekran znów gaśnie.
4. Alternatywnie w oknie **klikasz drugi raz** → `BT` (rozgłasza); jeśli nikt się nie połączy, po 5 min i tak sen.

> Wynik: przy słabej baterii urządzenie domyślnie śpi; budzisz je na chwilę jednym kliknięciem.

### 22.4. Przyciski — typowe gesty
- **Toggle BLE:** ekran świeci, klik → `BT`; klik → wyłączone.
- **Wybudzenie:** śpi, klik → ekran wstaje (to kliknięcie **nie** włącza rozgłaszania).
- **Uśpij teraz:** trzymasz **≥1,5 s** → ekran gaśnie natychmiast (zadziała w trakcie trzymania).
- *Wyjątek:* trzymanie przycisku, którym właśnie wybudziłeś, **nie** uśpi urządzenia (ochrona `wakePress`).

> Przykład sekwencji: śpi → klik (ekran on) → klik (BT on) → trzymasz 1,5 s (sen, ekran off, ale jeśli ktoś był połączony — łącze żyje do rozłączenia).

### 22.5. BLE — połączenie i podgląd
1. Otwierasz `nrf52_clock_console.html` w Chrome, klikasz „Połącz", wybierasz **nRF52-Clock**.
2. OLED zmienia `BT` → **`BT*`** (połączony). Strona co sekundę dostaje np.:
   `07-06-2026 14:32:07;T=23.5;H=45.2;B=87;V=3.85;CHG=0;R=-58`.
3. Widzisz zegar, datę, temperaturę, wilgotność, baterię, napięcie oraz **wykresy na żywo** (T/H/bateria/RSSI). Najechanie myszką pokazuje wartość i godzinę próbki.
4. Rozłączasz się → `BT*` wraca do `BT`, a na baterii rusza świeże **30 min** rozgłaszania.

### 22.6. BLE — ustawianie czasu (ręcznie)
1. Wpisujesz na stronie `07-06-2026 14:31:00` i klikasz „Wyślij".
2. OLED pokazuje **kod**, np. `2187` (ważny 60 s). Wewnętrznie czas zapisano jako UTC (`12:31:00` latem).
3. Wpisujesz `2187` w „Potwierdź" → zegar ustawiony, **sekundy ruszają od 0**.
- *Wyjątek:* zły kod → odrzucony; po 60 s → wygasa; zła data (np. `31-02-2026`) → odrzucona.

### 22.7. BLE — ustawianie czasu „na styk" (auto)
1. Klikasz „Pełna minuta ↑" → pole ustawia np. `14:32:00`.
2. „Wyślij" → kod na ekranie; wpisujesz go i klikasz **„Auto o godzinie"**.
3. Strona pilnuje zegara komputera i **sama wysyła potwierdzenie** dokładnie o `14:32:00` → urządzenie startuje równo na :00 (z dokładnością do opóźnienia BLE rzędu dziesiątek ms).

### 22.8. Sen z aktywnym połączeniem
1. Telefon połączony (`BT*`), trzymasz przycisk **1,5 s**.
2. Ekran gaśnie, rozgłaszanie staje, ale **połączenie żyje** — aplikacja dalej dostaje dane co sekundę mimo ciemnego ekranu, a czujnik jest **odświeżany** (2 s na USB / 60 s na baterii), bo ktoś słucha.
3. Gdy telefon się rozłączy → BLE w pełni gaśnie, urządzenie zostaje uśpione.

### 22.9. Nocne ściemnianie
1. O **20:00** (czasu lokalnego) kontrast OLED spada z `0x80` do `0x00` — ekran wyraźnie przygasa.
2. O **08:00** wraca do pełnej jasności. Komenda I2C idzie tylko w momencie zmiany.

### 22.10. Zmiana czasu letni/zimowy (DST)
- **Jesień:** ostatnia niedziela października — o lokalnej `03:00` zegar pokaże ponownie `02:00` (cofnięcie o godzinę).
- **Wiosna:** ostatnia niedziela marca — o lokalnej `02:00` zegar przeskoczy na `03:00`.
- Dzieje się **automatycznie**, bo czas trzymany jest w UTC; nawet jeśli urządzenie spało w chwili przełączenia, po wybudzeniu pokaże poprawną godzinę.

### 22.11. Głębokie rozładowanie → System OFF
1. Na baterii napięcie schodzi do **≤3,00 V** i utrzymuje się tak przez **3 kolejne odczyty (~15 s)**.
2. OLED pokazuje `LOW BATTERY / powering off`, po czym urządzenie wchodzi w **System OFF** (pobór ~µA), chroniąc ogniwo.
3. Klik budzi urządzenie, ale to **reset** — czas wraca do domyślnego i trzeba go ustawić ponownie przez BLE.

### 22.12. Wpięcie/wypięcie USB w trakcie pracy
- **Wpięcie:** natychmiastowy `readBattery()`, TX → +4 dBm, `CHG` na ekranie, brak limitu rozgłaszania.
- **Wypięcie:** TX → −4 dBm, powrót do `%`+napięcia, a jeśli rozgłaszanie było włączone — start świeżych **30 min** od chwili odpięcia.

---

## 23. Znane ograniczenia i pomysły

- **Brak trwałości czasu** po resecie/System OFF (rozważ zapis epoch do flasha lub zewnętrzny RTC z podtrzymaniem).
- **BLE otwarte** (`SECMODE_OPEN`) — brak parowania/szyfrowania.
- **Standardowe usługi** (Battery Service, Environmental Sensing, Current Time) ułatwiłyby integrację z generycznymi aplikacjami.
- **Najgłębszy sen CPU** (powiadomienia zadań FreeRTOS z ISR) dałby ~1 wybudzenie/s, ale jest bardziej inwazyjny niż obecne `delay()`.
- **Watchdog (WDT)** zwiększyłby odporność na zawieszenia.

---