/*
 * ╔══════════════════════════════════════════════════════════════╗
 *       ROCKET MOTOR STATIC TEST — TEENSY 4.1 (DAQ MASTER)
 * ╠══════════════════════════════════════════════════════════════╣
 *
 *  ROLE: High-speed sensor acquisition + SD card logging
 *        Streams live data to ESP32 over UART for Wi-Fi display
 *
 *  SENSORS:
 *   1. Thrust      — CZL-611RD 10kg load cell via HX711
 *   2. Temperature — K-type thermocouple via MAX6675
 *   3. Pressure    — 10 MPa transducer (0.5–4.5V ratiometric)
 *
 * ╠══════════════════════════════════════════════════════════════╣
 *  PIN CONNECTIONS — TEENSY 4.1
 * ──────────────────────────────────────────────────────────────
 *
 *  HX711 (Load Cell Amplifier):
 *    HX711 VCC  → Teensy 3.3V
 *    HX711 GND  → Teensy GND
 *    HX711 DT   → Teensy PIN 2
 *    HX711 SCK  → Teensy PIN 3
 *
 *  MAX6675 (Thermocouple Amplifier):
 *    MAX6675 VCC → Teensy 3.3V
 *    MAX6675 GND → Teensy GND
 *    MAX6675 SCK → Teensy PIN 13  (SPI0 SCK)
 *    MAX6675 CS  → Teensy PIN 10  (SPI0 CS)
 *    MAX6675 SO  → Teensy PIN 12  (SPI0 MISO)
 *    K-type +ve  → MAX6675 T+
 *    K-type -ve  → MAX6675 T-
 *
 *  Pressure Transducer (0.5–4.5V out, 5V supply):
 *    ⚠ VOLTAGE DIVIDER — 4.5V max > Teensy 3.3V ADC
 *    Transducer VCC (PIN1) → 5V
 *    Transducer GND (PIN2) → GND
 *    Transducer SIG (PIN3) → R1(10kΩ) → Teensy A0 (PIN14)
 *                                         also → R2(20kΩ) → GND
 *    Divider: 4.5V × 20/(10+20) = 3.0V ✓ safe for 3.3V ADC
 *
 *  ⚠ FLOATING PIN FIX (if sensor not connected):
 *    Add a 10kΩ resistor from Teensy A0 → GND
 *
 *  SD Card (built into Teensy 4.1 — use the onboard slot):
 *    Just insert a FAT32 formatted microSD card.
 *    No extra wiring needed.
 *
 *  UART to ESP32 (Serial1):
 *    Teensy PIN 1  (TX1) → ESP32 GPIO 16 (RX2)
 *    Teensy PIN 0  (RX1) → ESP32 GPIO 17 (TX2)
 *    Teensy GND          → ESP32 GND  (MUST share ground)
 *
 *  Record Button (optional physical trigger):
 *    One leg → Teensy PIN 4
 *    Other leg → GND
 *    (Internal pull-up enabled in code)
 *
 * ╠══════════════════════════════════════════════════════════════╣
 *  SD CARD:
 *    Format: FAT32
 *    File:   /daq_NNNN.csv  (auto-increments each power-on)
 *    Columns: time_ms, force_N, force_kg, force_N_smooth,
 *             force_kg_smooth, temp_C, pressure_MPa
 *
 *  SERIAL STREAM TO ESP32 (Serial1):
 *    Baud: 115200
 *    Format (one line per sample):
 *    $F:12.340,FK:1.2579,T:28.50,P:1.2340,I:45.600,PK:67.800,PKK:6.9104,RC:123,HD:0,REC:0\n
 *
 *  COMMANDS FROM ESP32 → Teensy (Serial1 RX):
 *    "TARE\n"          — re-tare load cell, reset peaks & impulse
 *    "REC_ON\n"        — start recording
 *    "REC_OFF\n"       — stop recording
 *    "RESETPEAK\n"     — clear peak values and impulse
 *    "CAL:xxxxxx.xx\n" — update load cell calibration factor
 *    "FREQ:xx\n"       — update UART send period in ms (10–1000)
 *
 *  LIBRARIES:
 *    - HX711           by Bogdan Necula
 *    - MAX6675 library by Adafruit
 *    - SD              built-in
 * ╚══════════════════════════════════════════════════════════════╝
 */

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <HX711.h>
#include <max6675.h>

// ── Pins ─────────────────────────────────────────────────────
#define HX_DT        2
#define HX_SCK       3
#define TC_CS        10
#define TC_SCK       13
#define TC_SO        12
#define PRES_PIN     A0
#define BTN_PIN      4
#define SD_CS        BUILTIN_SDCARD

// ── UART to ESP32 ────────────────────────────────────────────
#define ESP_SERIAL   Serial1
#define ESP_BAUD     115200

// ── Calibration — runtime-editable via CAL: command ──────────
float    THRUST_CAL      = 107809.50451f;

// ── Pressure ─────────────────────────────────────────────────
const float ADC_VREF   = 3.3f;
const float ADC_COUNTS = 4095.0f;
const float VD_RATIO   = 20.0f / 30.0f;
const float P_V_MIN    = 0.5f;
const float P_V_SPAN   = 4.0f;
const float P_MAX_MPa  = 10.0f;
#define PRESSURE_CONNECTED   false

// ── Smoothing ────────────────────────────────────────────────
#define NOISE_FLOOR_KG   0.008f
#define EMA_ALPHA        0.6f

// ── Sample rate targets ───────────────────────────────────────
#define LOOP_PERIOD_US   2000
#define TEMP_PERIOD_MS   250

// ── UART send period — runtime-editable via FREQ: command ────
//    10 ms = 100 Hz  |  50 ms = 20 Hz (default)  |  100 ms = 10 Hz
uint32_t UART_PERIOD_MS  = 50;

// ── Buffer ────────────────────────────────────────────────────
#define MAX_SAMPLES  50000

struct Sample {
  uint32_t time_ms;
  float    force_N;
  float    force_kg;
  float    force_N_smooth;
  float    force_kg_smooth;
  float    temp_C;
  float    pressure_MPa;
};
EXTMEM Sample recBuf[MAX_SAMPLES];

// ── State ─────────────────────────────────────────────────────
uint32_t recCount       = 0;
bool     isRecording    = false;
bool     hasData        = false;
uint32_t recStartMs     = 0;

float    liveForce_N     = 0.0f;
float    liveForce_kg    = 0.0f;
float    liveForceRaw_N  = 0.0f;
float    liveForceRaw_kg = 0.0f;
float    livePeak_N      = 0.0f;
float    livePeak_kg     = 0.0f;
float    liveTemp_C      = 0.0f;
float    livePres_MPa    = 0.0f;
float    liveImpulse_Ns  = 0.0f;

float    smoothKg        = 0.0f;
bool     emaInit         = false;

// ── Objects ───────────────────────────────────────────────────
HX711   scale;
MAX6675 tc(TC_SCK, TC_CS, TC_SO);

// ── SD file ───────────────────────────────────────────────────
File    sdFile;
bool    sdReady          = false;
char    sdFileName[24];

// ── Timing ────────────────────────────────────────────────────
uint32_t lastLoopUs    = 0;
uint32_t lastTempMs    = 0;
uint32_t lastSampleMs  = 0;
uint32_t lastUartMs    = 0;

// ─────────────────────────────────────────────────────────────
float readPressure() {
  if (!PRESSURE_CONNECTED) return 0.0f;
  uint32_t sum = 0;
  for (int i = 0; i < 16; i++) sum += analogRead(PRES_PIN);
  float raw   = sum / 16.0f;
  float v_adc = raw * (ADC_VREF / ADC_COUNTS);
  float v_sig = v_adc / VD_RATIO;
  float p     = (v_sig - P_V_MIN) / P_V_SPAN * P_MAX_MPa;
  return constrain(p, 0.0f, P_MAX_MPa);
}

void openSDFile() {
  for (int i = 1; i <= 9999; i++) {
    snprintf(sdFileName, sizeof(sdFileName), "/daq_%04d.csv", i);
    if (!SD.exists(sdFileName)) {
      sdFile = SD.open(sdFileName, FILE_WRITE);
      if (sdFile) {
        sdFile.println("time_ms,force_N,force_kg,force_N_smooth,force_kg_smooth,temp_C,pressure_MPa");
        sdFile.flush();
        Serial.printf("SD file: %s\n", sdFileName);
      }
      return;
    }
  }
}

// ─────────────────────────────────────────────────────────────
void setup() {
  // ── UART to ESP32 first ───────────────────────────────────
  ESP_SERIAL.begin(ESP_BAUD);

  // ── USB Serial ───────────────────────────────────────────
  Serial.begin(115200);
  delay(800);

  Serial.println("\n╔══════════════════════╗");
  Serial.println("║  Teensy 4.1  DAQ    ║");
  Serial.println("╚══════════════════════╝");
  Serial.println("UART → Serial1 (pin1=TX, pin0=RX) @ 115200");

  pinMode(BTN_PIN, INPUT_PULLUP);
  analogReadResolution(12);

  // ── HX711 ─────────────────────────────────────────────────
  scale.begin(HX_DT, HX_SCK);
  Serial.print("HX711");
  uint32_t hxStart = millis();
  while (!scale.is_ready()) {
    Serial.print(".");
    delay(50);
    if (millis() - hxStart > 3000) {
      Serial.println(" TIMEOUT! Check DT→PIN2 SCK→PIN3 wiring.");
      break;
    }
  }
  if (scale.is_ready()) {
    scale.set_scale(THRUST_CAL);
    scale.tare();
    Serial.println(" OK, tared.");
  }

  // ── MAX6675 ───────────────────────────────────────────────
  delay(500);
  liveTemp_C = tc.readCelsius();
  Serial.printf("MAX6675 OK. Temp: %.1f C\n", liveTemp_C);
  lastTempMs = millis();

  // ── Pressure ──────────────────────────────────────────────
  livePres_MPa = readPressure();
  if (PRESSURE_CONNECTED)
    Serial.printf("Pressure OK. %.3f MPa\n", livePres_MPa);
  else
    Serial.println("Pressure sensor: NOT CONNECTED");

  // ── SD card ───────────────────────────────────────────────
  if (SD.begin(SD_CS)) {
    sdReady = true;
    openSDFile();
    Serial.println("SD card OK.");
  } else {
    Serial.println("SD card FAILED — logging to RAM only.");
  }

  Serial.printf("UART period: %lu ms (%lu Hz)\n",
                UART_PERIOD_MS, 1000UL / UART_PERIOD_MS);
  Serial.printf("Cal factor:  %.5f\n", THRUST_CAL);
  Serial.println("Ready. Press button or send REC_ON from phone.");
  Serial.println("══════════════════════");

  lastLoopUs   = micros();
  lastSampleMs = millis();
}

// ─────────────────────────────────────────────────────────────
void loop() {
  // ── Rate-limit main loop ──────────────────────────────────
  while ((micros() - lastLoopUs) < LOOP_PERIOD_US) {}
  lastLoopUs = micros();
  uint32_t now = millis();

  // ── Physical record button ────────────────────────────────
  static bool lastBtn = HIGH;
  bool curBtn = digitalRead(BTN_PIN);
  if (lastBtn == HIGH && curBtn == LOW) {
    if (!isRecording) {
      recCount = 0; hasData = false; liveImpulse_Ns = 0;
      lastSampleMs = now; recStartMs = now;
      isRecording = true;
      if (sdReady) {
        if (sdFile) sdFile.close();
        openSDFile();
      }
      Serial.println("▶ RECORDING STARTED");
    } else {
      isRecording = false;
      hasData = (recCount > 0);
      if (sdFile) { sdFile.flush(); sdFile.close(); }
      Serial.printf("⏹ RECORDING STOPPED — %lu samples\n", recCount);
    }
    delay(50);
  }
  lastBtn = curBtn;

  // ── Read thrust ───────────────────────────────────────────
  if (scale.is_ready()) {
    float kg = scale.get_units(1);
    float clamped = (kg < NOISE_FLOOR_KG) ? 0.0f : kg;

    if (!emaInit) { smoothKg = clamped; emaInit = true; }
    else smoothKg = (EMA_ALPHA * clamped) + ((1.0f - EMA_ALPHA) * smoothKg);

    liveForceRaw_kg = clamped;
    liveForceRaw_N  = clamped * 9.81f;
    liveForce_kg    = smoothKg;
    liveForce_N     = smoothKg * 9.81f;

    if (liveForceRaw_N  > livePeak_N)  livePeak_N  = liveForceRaw_N;
    if (liveForceRaw_kg > livePeak_kg) livePeak_kg = liveForceRaw_kg;
  }

  // ── Read pressure ─────────────────────────────────────────
  livePres_MPa = readPressure();

  // ── Read temperature every 250 ms ────────────────────────
  if (now - lastTempMs >= TEMP_PERIOD_MS) {
    float t = tc.readCelsius();
    if (!isnan(t) && t > -100.0f) liveTemp_C = t;
    lastTempMs = now;
  }

  // ── Recording to RAM buffer + SD ─────────────────────────
  if (isRecording) {
    float dt = (now - lastSampleMs) / 1000.0f;
    liveImpulse_Ns += liveForce_N * dt;
    lastSampleMs = now;
    uint32_t elapsed = now - recStartMs;

    if (recCount < MAX_SAMPLES) {
      recBuf[recCount] = {
        elapsed,
        liveForceRaw_N,
        liveForceRaw_kg,
        liveForce_N,
        liveForce_kg,
        liveTemp_C,
        livePres_MPa
      };
      recCount++;
    } else {
      isRecording = false; hasData = true;
      if (sdFile) { sdFile.flush(); sdFile.close(); }
      Serial.println("Buffer full — auto-stopped.");
    }

    if (sdReady && sdFile) {
      char row[96];
      snprintf(row, sizeof(row), "%lu,%.4f,%.4f,%.4f,%.4f,%.2f,%.4f\r\n",
        (unsigned long)elapsed,
        liveForceRaw_N, liveForceRaw_kg,
        liveForce_N,    liveForce_kg,
        liveTemp_C,     livePres_MPa);
      sdFile.print(row);
      if (recCount % 200 == 0) sdFile.flush();
    }
  } else {
    lastSampleMs = now;
  }

  // ── Send to ESP32 via Serial1 ─────────────────────────────
  if (now - lastUartMs >= UART_PERIOD_MS) {
    ESP_SERIAL.printf(
      "$F:%.3f,FK:%.4f,T:%.2f,P:%.4f,I:%.3f,PK:%.3f,PKK:%.4f,RC:%lu,HD:%d,REC:%d\n",
      liveForce_N,    liveForce_kg,
      liveTemp_C,     livePres_MPa,
      liveImpulse_Ns,
      livePeak_N,     livePeak_kg,
      recCount,       (int)hasData,  (int)isRecording);
    lastUartMs = now;
  }

  // ── USB Serial monitor ────────────────────────────────────
  if (now % 100 < 2) {
    Serial.printf(
      "Raw:%.4fkg/%.3fN | Smooth:%.4fkg/%.3fN | Peak:%.4fkg | T:%.1fC | P:%.3fMPa | Imp:%.2fNs | Rec:%s | Pts:%lu\n",
      liveForceRaw_kg, liveForceRaw_N,
      liveForce_kg,    liveForce_N,
      livePeak_kg,
      liveTemp_C,      livePres_MPa,
      liveImpulse_Ns,
      isRecording ? "●" : "○",
      recCount);
  }

  // ── Commands from ESP32 via Serial1 RX ───────────────────
  while (ESP_SERIAL.available()) {
    String cmd = ESP_SERIAL.readStringUntil('\n');
    cmd.trim();

    if (cmd == "TARE") {
      scale.tare();
      livePeak_N = 0; livePeak_kg = 0;
      liveImpulse_Ns = 0;
      smoothKg = 0; emaInit = false;
      Serial.println("✓ Tared via ESP32.");

    } else if (cmd == "REC_ON" && !isRecording) {
      recCount = 0; hasData = false; liveImpulse_Ns = 0;
      lastSampleMs = now; recStartMs = now;
      isRecording = true;
      if (sdReady) { if (sdFile) sdFile.close(); openSDFile(); }
      Serial.println("▶ RECORDING via ESP32");

    } else if (cmd == "REC_OFF" && isRecording) {
      isRecording = false; hasData = (recCount > 0);
      if (sdFile) { sdFile.flush(); sdFile.close(); }
      Serial.printf("⏹ STOPPED via ESP32 — %lu samples\n", recCount);

    } else if (cmd == "RESETPEAK") {
      livePeak_N = 0; livePeak_kg = 0; liveImpulse_Ns = 0;
      Serial.println("✓ Peaks reset via ESP32.");

    // ── NEW: update load cell calibration factor ──────────
    // Format: "CAL:107809.50000"
    } else if (cmd.startsWith("CAL:")) {
      float newCal = cmd.substring(4).toFloat();
      if (newCal > 100.0f) {
        THRUST_CAL = newCal;
        scale.set_scale(THRUST_CAL);
        Serial.printf("✓ CAL updated → %.5f\n", THRUST_CAL);
        ESP_SERIAL.printf("ACK:CAL:%.5f\n", THRUST_CAL);
      } else {
        Serial.printf("✗ CAL rejected (too small): %s\n", cmd.c_str());
      }

    // ── NEW: update UART send frequency ───────────────────
    // Format: "FREQ:50"  (value is period in ms)
    } else if (cmd.startsWith("FREQ:")) {
      uint32_t newPeriod = (uint32_t)cmd.substring(5).toInt();
      if (newPeriod >= 10 && newPeriod <= 1000) {
        UART_PERIOD_MS = newPeriod;
        Serial.printf("✓ UART period → %lu ms (%lu Hz)\n",
                      UART_PERIOD_MS, 1000UL / UART_PERIOD_MS);
        ESP_SERIAL.printf("ACK:FREQ:%lu\n", UART_PERIOD_MS);
      } else {
        Serial.printf("✗ FREQ rejected (out of 10–1000): %s\n", cmd.c_str());
      }
    }
  }
}

/*
 ╔══════════════════════════════════════════════════════════════╗
 ║  CALIBRATION                                                ║
 ╠══════════════════════════════════════════════════════════════╣
 ║  THRUST (one-time physical calibration):                    ║
 ║   1. Set THRUST_CAL = 1.0 and flash                         ║
 ║   2. Place known weight (1 kg) on load cell                 ║
 ║   3. Read raw value in Serial Monitor                       ║
 ║   4. THRUST_CAL = raw / weight_kg                           ║
 ║   5. Re-flash with new value                                ║
 ║   — OR — use the phone UI to send CAL: live                 ║
 ║                                                             ║
 ║  PRESSURE:                                                  ║
 ║   Set PRESSURE_CONNECTED = true when sensor is wired        ║
 ║                                                             ║
 ║  UART WIRING:                                               ║
 ║   Teensy PIN 1 (TX1) → ESP32 GPIO 16 (RX2)                 ║
 ║   Teensy PIN 0 (RX1) → ESP32 GPIO 17 (TX2)                 ║
 ║   Teensy GND         → ESP32 GND  ← CRITICAL               ║
 ╚══════════════════════════════════════════════════════════════╝
*/
