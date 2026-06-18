/**
 * ============================================================
 *  NGSRP — Arduino Uno | PRIMARY SENSOR NODE (Transmitter)
 * ============================================================
 *  Next-Gen Smart Railway Platform
 *  Detects train approach/departure using:
 *    • Hall Effect Sensor 1  → D2  (magnetic field / wheel detection)
 *    • Hall Effect Sensor 2  → D3  (magnetic field / wheel detection)
 *    • Vibration Sensor      → A0  (track vibration)
 *
 *  Transmits a 6-byte payload via NRF24L01 to the bridge controller.
 *  Payload format (6 bytes):
 *    [0]   Hour   (BCD, 24-hr)
 *    [1]   Minute (BCD)
 *    [2]   Second (BCD)
 *    [3]   Status  → 0x01 = APPROACHING | 0x10 = DEPARTING | 0x00 = IDLE
 *    [4]   Sensor flags (bit0=Hall1, bit1=Hall2, bit2=Vibration)
 *    [5]   Checksum (XOR of bytes 0-4)
 *
 *  NRF24L01 Wiring (Arduino Uno):
 *    VCC  → 3.3 V
 *    GND  → GND
 *    CE   → D9
 *    CSN  → D10
 *    SCK  → D13
 *    MOSI → D11
 *    MISO → D12
 *
 *  Library requirements:
 *    - RF24 by TMRh20   (Install via Arduino Library Manager)
 *
 *  Author : NGSRP Team — Sri Sai Ram Engineering College
 *  Version: 1.0.0
 * ============================================================
 */

#include <SPI.h>
#include <RF24.h>
#ifdef printf_P
  #undef printf_P
#endif

// ─────────────────────────────────────────────
//  Pin Definitions
// ─────────────────────────────────────────────
#define HALL_SENSOR_1_PIN   2       // Digital input — Hall Effect Sensor 1
#define HALL_SENSOR_2_PIN   3       // Digital input — Hall Effect Sensor 2
#define VIBRATION_PIN       A0      // Analog input  — Vibration Sensor

#define NRF_CE_PIN          9
#define NRF_CSN_PIN         10

// ─────────────────────────────────────────────
//  NRF24L01 Configuration
// ─────────────────────────────────────────────
RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);

// 5-byte address — must match receiver exactly
const byte PIPE_ADDRESS[6] = "NGSRP";

// ─────────────────────────────────────────────
//  Sensor Thresholds & Timing
// ─────────────────────────────────────────────
#define VIBRATION_THRESHOLD     450   // Raw ADC value (0-1023) above which train is detected
#define DEBOUNCE_MS             50    // Hall sensor debounce delay in milliseconds
#define TRAIN_CONFIRM_WINDOW_MS 3000  // Time window to confirm multi-sensor agreement
#define IDLE_TIMEOUT_MS         8000  // After this long with no detection, go back to IDLE
#define TRANSMIT_INTERVAL_MS    500   // How often to re-broadcast while active

// ─────────────────────────────────────────────
//  Train Status Codes
// ─────────────────────────────────────────────
#define STATUS_IDLE         0x00
#define STATUS_APPROACHING  0x01
#define STATUS_DEPARTING    0x10

// ─────────────────────────────────────────────
//  Globals
// ─────────────────────────────────────────────
uint8_t  currentStatus    = STATUS_IDLE;
uint32_t lastDetectTime   = 0;
uint32_t lastTransmitTime = 0;
bool     hall1PrevState   = HIGH;
bool     hall2PrevState   = HIGH;
uint32_t hall1TriggerTime = 0;
uint32_t hall2TriggerTime = 0;

// Simple software clock (seconds since boot — replace with RTC if available)
uint32_t bootEpoch        = 0;   // Set via Serial command: "TIME:HH:MM:SS"
uint32_t bootMillis       = 0;

// ─────────────────────────────────────────────
//  Forward Declarations
// ─────────────────────────────────────────────
void     readSensors(bool &hall1, bool &hall2, bool &vibDetected);
void     determineStatus(bool hall1, bool hall2, bool vibDetected);
void     transmitPayload();
void     buildPayload(uint8_t *buf);
void     getCurrentTime(uint8_t &h, uint8_t &m, uint8_t &s);
void     handleSerialCommand();
uint8_t  computeChecksum(uint8_t *buf, uint8_t len);

// ─────────────────────────────────────────────
//  Setup
// ─────────────────────────────────────────────
void setup() {
  Serial.begin(9600);
  Serial.println(F("=== NGSRP Sensor Node — Transmitter ==="));

  // Sensor pins
  pinMode(HALL_SENSOR_1_PIN, INPUT_PULLUP);
  pinMode(HALL_SENSOR_2_PIN, INPUT_PULLUP);
  // VIBRATION_PIN is analog — no pinMode needed

  // NRF24L01 initialisation
  if (!radio.begin()) {
    Serial.println(F("[ERROR] NRF24L01 not detected. Check wiring!"));
    while (true) { delay(1000); }
  }

  radio.setChannel(76);                 // Channel 76 (2.476 GHz) — away from WiFi congestion
  radio.setPALevel(RF24_PA_HIGH);       // Increase range (use RF24_PA_LOW for bench testing)
  radio.setDataRate(RF24_250KBPS);      // Longer range at lower data rate
  radio.setPayloadSize(6);              // Fixed 6-byte payload
  radio.setCRCLength(RF24_CRC_16);      // 16-bit CRC for reliability
  radio.openWritingPipe(PIPE_ADDRESS);
  radio.stopListening();                // Transmitter role

  Serial.println(F("[OK] NRF24L01 ready — Transmitter mode"));
  Serial.println(F("[INFO] Send TIME:HH:MM:SS via Serial to set clock"));
  Serial.println(F("[INFO] Monitoring sensors..."));

  bootMillis = millis();
}

// ─────────────────────────────────────────────
//  Main Loop
// ─────────────────────────────────────────────
void loop() {
  // Handle optional Serial time-set command
  if (Serial.available()) {
    handleSerialCommand();
  }

  bool hall1Detected, hall2Detected, vibDetected;
  readSensors(hall1Detected, hall2Detected, vibDetected);
  determineStatus(hall1Detected, hall2Detected, vibDetected);

  uint32_t now = millis();

  // Transmit at regular interval while active, or once on status change
  if (currentStatus != STATUS_IDLE) {
    if (now - lastTransmitTime >= TRANSMIT_INTERVAL_MS) {
      transmitPayload();
      lastTransmitTime = now;
    }
  }

  // Auto-reset to IDLE after timeout
  if (currentStatus != STATUS_IDLE &&
      (now - lastDetectTime > IDLE_TIMEOUT_MS)) {
    Serial.println(F("[INFO] No detection — returning to IDLE"));
    currentStatus = STATUS_IDLE;
    transmitPayload();   // Send one IDLE packet to notify receiver
  }

  delay(50);  // Small loop delay to reduce CPU noise
}

// ─────────────────────────────────────────────
//  Sensor Reading
// ─────────────────────────────────────────────
void readSensors(bool &hall1, bool &hall2, bool &vibDetected) {
  // Hall sensors are active-LOW (INPUT_PULLUP — triggered when magnet present)
  bool h1 = (digitalRead(HALL_SENSOR_1_PIN) == LOW);
  bool h2 = (digitalRead(HALL_SENSOR_2_PIN) == LOW);

  // Debounce Hall 1
  if (h1 && !hall1PrevState) {
    delay(DEBOUNCE_MS);
    h1 = (digitalRead(HALL_SENSOR_1_PIN) == LOW);
    if (h1) hall1TriggerTime = millis();
  }
  hall1PrevState = h1;

  // Debounce Hall 2
  if (h2 && !hall2PrevState) {
    delay(DEBOUNCE_MS);
    h2 = (digitalRead(HALL_SENSOR_2_PIN) == LOW);
    if (h2) hall2TriggerTime = millis();
  }
  hall2PrevState = h2;

  // Vibration sensor — analog threshold check (average 5 readings)
  int vibSum = 0;
  for (int i = 0; i < 5; i++) {
    vibSum += analogRead(VIBRATION_PIN);
    delay(2);
  }
  int vibAvg = vibSum / 5;
  vibDetected = (vibAvg > VIBRATION_THRESHOLD);

  hall1 = h1;
  hall2 = h2;

  // Debug output
  if (h1 || h2 || vibDetected) {
    Serial.print(F("[SENSOR] H1="));
    Serial.print(h1);
    Serial.print(F(" H2="));
    Serial.print(h2);
    Serial.print(F(" Vib="));
    Serial.print(vibAvg);
    Serial.print(F(" (thresh="));
    Serial.print(VIBRATION_THRESHOLD);
    Serial.println(F(")"));
  }
}

// ─────────────────────────────────────────────
//  Status Determination Logic
//  Train direction inferred from Hall sensor
//  trigger order:
//    Hall1 first → Hall2 second = APPROACHING
//    Hall2 first → Hall1 second = DEPARTING
// ─────────────────────────────────────────────
void determineStatus(bool hall1, bool hall2, bool vibDetected) {
  uint32_t now = millis();

  // Require vibration + at least one Hall for confident detection
  if (!vibDetected && !hall1 && !hall2) return;

  lastDetectTime = now;

  if (hall1 && hall2) {
    // Both triggered — determine order by timestamp
    uint32_t gap = (hall1TriggerTime > hall2TriggerTime)
                   ? (hall1TriggerTime - hall2TriggerTime)
                   : (hall2TriggerTime - hall1TriggerTime);

    if (gap < TRAIN_CONFIRM_WINDOW_MS) {
      if (hall1TriggerTime <= hall2TriggerTime) {
        if (currentStatus != STATUS_APPROACHING) {
          currentStatus = STATUS_APPROACHING;
          Serial.println(F("[STATUS] TRAIN APPROACHING — H1 triggered before H2"));
        }
      } else {
        if (currentStatus != STATUS_DEPARTING) {
          currentStatus = STATUS_DEPARTING;
          Serial.println(F("[STATUS] TRAIN DEPARTING — H2 triggered before H1"));
        }
      }
    }
  } else if (hall1 || vibDetected) {
    // Partial detection — assume approaching if we don't have direction info
    if (currentStatus == STATUS_IDLE) {
      currentStatus = STATUS_APPROACHING;
      Serial.println(F("[STATUS] Partial detection → APPROACHING (confirm pending)"));
    }
  }
}

// ─────────────────────────────────────────────
//  Build & Transmit Payload
// ─────────────────────────────────────────────
void transmitPayload() {
  uint8_t payload[6];
  buildPayload(payload);

  bool success = radio.write(payload, sizeof(payload));

  Serial.print(F("[TX] Status=0x"));
  Serial.print(payload[3], HEX);
  Serial.print(F(" Time="));
  Serial.print(payload[0]);
  Serial.print(F(":"));
  Serial.print(payload[1]);
  Serial.print(F(":"));
  Serial.print(payload[2]);
  Serial.print(F(" → "));
  Serial.println(success ? F("OK") : F("FAILED"));
}

void buildPayload(uint8_t *buf) {
  uint8_t h, m, s;
  getCurrentTime(h, m, s);

  buf[0] = h;
  buf[1] = m;
  buf[2] = s;
  buf[3] = currentStatus;

  // Sensor flags
  buf[4]  = (digitalRead(HALL_SENSOR_1_PIN) == LOW) ? 0x01 : 0x00;
  buf[4] |= (digitalRead(HALL_SENSOR_2_PIN) == LOW) ? 0x02 : 0x00;
  buf[4] |= (analogRead(VIBRATION_PIN) > VIBRATION_THRESHOLD) ? 0x04 : 0x00;

  buf[5] = computeChecksum(buf, 5);
}

// ─────────────────────────────────────────────
//  Simple Clock — based on millis() offset
//  Set via Serial: TIME:HH:MM:SS
// ─────────────────────────────────────────────
void getCurrentTime(uint8_t &h, uint8_t &m, uint8_t &s) {
  uint32_t elapsed = (millis() - bootMillis) / 1000UL;
  uint32_t totalSec = bootEpoch + elapsed;

  s = totalSec % 60;
  m = (totalSec / 60) % 60;
  h = (totalSec / 3600) % 24;
}

void handleSerialCommand() {
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();

  // Format: TIME:HH:MM:SS
  if (cmd.startsWith("TIME:") && cmd.length() == 13) {
    uint8_t h = cmd.substring(5, 7).toInt();
    uint8_t m = cmd.substring(8, 10).toInt();
    uint8_t s = cmd.substring(11, 13).toInt();

    if (h < 24 && m < 60 && s < 60) {
      bootEpoch  = (uint32_t)h * 3600 + (uint32_t)m * 60 + s;
      bootMillis = millis();
      Serial.print(F("[CLOCK] Set to "));
      Serial.print(h); Serial.print(F(":")); Serial.print(m); Serial.print(F(":")); Serial.println(s);
    } else {
      Serial.println(F("[ERROR] Invalid time values"));
    }
  } else if (cmd == "STATUS") {
    Serial.print(F("[INFO] Current status: 0x"));
    Serial.println(currentStatus, HEX);
  } else if (cmd == "RESET") {
    currentStatus = STATUS_IDLE;
    Serial.println(F("[INFO] Status reset to IDLE"));
  } else {
    Serial.println(F("[INFO] Commands: TIME:HH:MM:SS | STATUS | RESET"));
  }
}

// ─────────────────────────────────────────────
//  Checksum — XOR of all bytes
// ─────────────────────────────────────────────
uint8_t computeChecksum(uint8_t *buf, uint8_t len) {
  uint8_t cs = 0;
  for (uint8_t i = 0; i < len; i++) cs ^= buf[i];
  return cs;
}
