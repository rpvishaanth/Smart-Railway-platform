/**
 * ============================================================
 *  NGSRP — ESP8266 (NodeMCU) | BRIDGE CONTROLLER (Receiver)
 * ============================================================
 *  Next-Gen Smart Railway Platform
 *
 *  This node:
 *    1. Receives NRF24L01 packets from the sensor Arduino
 *    2. Validates the payload checksum
 *    3. Controls a servo motor to open/close the mobile bridge
 *    4. Connects to WiFi and hosts a real-time web dashboard
 *       at the ESP8266's local IP address
 *
 *  Servo:
 *    0°   → Bridge CLOSED  (train passage allowed)
 *    90°  → Bridge OPEN    (pedestrian crossing enabled)
 *
 *  NRF24L01 Wiring (NodeMCU / ESP8266):
 *    VCC  → 3.3 V  (use a 10µF cap across VCC/GND for stability)
 *    GND  → GND
 *    CE   → D4  (GPIO 2)
 *    CSN  → D8  (GPIO 15)
 *    SCK  → D5  (GPIO 14)
 *    MOSI → D7  (GPIO 13)
 *    MISO → D6  (GPIO 12)
 *
 *  Servo Wiring:
 *    Signal → D3  (GPIO 0)
 *    VCC    → 5 V (external — NodeMCU 3.3 V cannot drive servos reliably)
 *    GND    → GND (common with NodeMCU)
 *
 *  Library requirements (install via Arduino Library Manager):
 *    - RF24 by TMRh20
 *    - Servo (built-in)
 *    - ESP8266WiFi (included with ESP8266 board package)
 *    - ESP8266WebServer (included with ESP8266 board package)
 *
 *  Board setting: "NodeMCU 1.0 (ESP-12E Module)"
 * ============================================================
 */

#include <SPI.h>
#include <RF24.h>
#ifdef printf_P
  #undef printf_P
#endif
#include <Servo.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>


// ─────────────────────────────────────────────
//  WiFi Credentials — change before upload
// ─────────────────────────────────────────────
const char* WIFI_SSID     = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";

// ─────────────────────────────────────────────
//  Pin Definitions
// ─────────────────────────────────────────────
#define NRF_CE_PIN    2    // D4 on NodeMCU
#define NRF_CSN_PIN   15   // D8 on NodeMCU
#define SERVO_PIN     0    // D3 on NodeMCU

// ─────────────────────────────────────────────
//  Servo Angles
// ─────────────────────────────────────────────
#define BRIDGE_OPEN_ANGLE   90   // Degrees — bridge extended for pedestrians
#define BRIDGE_CLOSED_ANGLE  0   // Degrees — bridge retracted for train

// ─────────────────────────────────────────────
//  Status Codes (must match transmitter)
// ─────────────────────────────────────────────
#define STATUS_IDLE         0x00
#define STATUS_APPROACHING  0x01
#define STATUS_DEPARTING    0x10

// ─────────────────────────────────────────────
//  NRF24L01 & Servo
// ─────────────────────────────────────────────
RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);
const byte PIPE_ADDRESS[6] = "NGSRP";

Servo bridgeServo;

// ─────────────────────────────────────────────
//  Web Server on port 80
// ─────────────────────────────────────────────
ESP8266WebServer server(80);

// ─────────────────────────────────────────────
//  State Variables
// ─────────────────────────────────────────────
uint8_t  lastStatus        = STATUS_IDLE;
bool     bridgeOpen        = false;
bool     manualOverride    = false;   // Web UI manual control flag
uint32_t lastPacketTime    = 0;
uint32_t lastStatusChange  = 0;
uint8_t  lastPayload[6]    = {0};
char     lastTimeStr[9]    = "00:00:00";
int      packetCount       = 0;
int      errorCount        = 0;

// Bridge operation log (last 10 events, ring buffer)
struct LogEntry {
  char     time[9];
  uint8_t  status;
  bool     bridgeWasOpen;
  bool     manual;
};
#define LOG_SIZE 10
LogEntry eventLog[LOG_SIZE];
int logHead = 0;

// ─────────────────────────────────────────────
//  Forward Declarations
// ─────────────────────────────────────────────
void     setupRadio();
void     setupWiFi();
void     setupWebServer();
void     receivePacket();
void     processStatus(uint8_t status, uint8_t *payload);
void     openBridge(bool manual = false);
void     closeBridge(bool manual = false);
void     addLog(uint8_t status, bool open, bool manual);
bool     verifyChecksum(uint8_t *buf);

// Web handlers
void     handleRoot();
void     handleStatus();
void     handleBridgeOpen();
void     handleBridgeClose();
void     handleReset();
void     handleNotFound();

String   buildStatusJSON();
String   buildDashboardHTML();

// ─────────────────────────────────────────────
//  Setup
// ─────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println(F("\n=== NGSRP Bridge Controller — ESP8266 ==="));

  // Servo
  bridgeServo.attach(SERVO_PIN);
  closeBridge();  // Start with bridge closed (safe default)
  Serial.println(F("[SERVO] Initialised — Bridge CLOSED"));

  // Radio
  setupRadio();

  // WiFi + Web server
  setupWiFi();
  setupWebServer();

  Serial.println(F("[READY] System online. Listening for sensor packets..."));
  Serial.print(F("[WEB] Dashboard: http://"));
  Serial.println(WiFi.localIP());
}

// ─────────────────────────────────────────────
//  Main Loop
// ─────────────────────────────────────────────
void loop() {
  // Handle incoming web requests
  server.handleClient();

  // Check for NRF24 packet (non-blocking)
  if (!manualOverride) {
    receivePacket();
  }

  // Safety: if no packet received for 30 seconds and bridge is open, close it
  if (bridgeOpen && !manualOverride &&
      millis() - lastPacketTime > 30000UL &&
      lastPacketTime != 0) {
    Serial.println(F("[SAFETY] Packet timeout — closing bridge"));
    closeBridge();
  }

  delay(10);
}

// ─────────────────────────────────────────────
//  NRF24L01 Setup
// ─────────────────────────────────────────────
void setupRadio() {
  if (!radio.begin()) {
    Serial.println(F("[ERROR] NRF24L01 not found! Check SPI wiring."));
    // Continue without radio — web control still available
    return;
  }

  radio.setChannel(76);
  radio.setPALevel(RF24_PA_HIGH);
  radio.setDataRate(RF24_250KBPS);
  radio.setPayloadSize(6);
  radio.setCRCLength(RF24_CRC_16);
  radio.openReadingPipe(1, PIPE_ADDRESS);
  radio.startListening();  // Receiver role

  Serial.println(F("[OK] NRF24L01 ready — Receiver mode (Ch 76)"));
}

// ─────────────────────────────────────────────
//  WiFi Setup
// ─────────────────────────────────────────────
void setupWiFi() {
  Serial.print(F("[WiFi] Connecting to: "));
  Serial.println(WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(F("."));
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.print(F("[WiFi] Connected! IP: "));
    Serial.println(WiFi.localIP());
  } else {
    Serial.println(F("\n[WiFi] Connection failed — running in offline mode"));
    // Start Access Point as fallback
    WiFi.mode(WIFI_AP);
    WiFi.softAP("NGSRP-Bridge", "railway123");
    Serial.print(F("[WiFi] AP started: NGSRP-Bridge | IP: "));
    Serial.println(WiFi.softAPIP());
  }
}

// ─────────────────────────────────────────────
//  Web Server Routes
// ─────────────────────────────────────────────
void setupWebServer() {
  server.on("/",            HTTP_GET,  handleRoot);
  server.on("/api/status",  HTTP_GET,  handleStatus);
  server.on("/api/open",    HTTP_POST, handleBridgeOpen);
  server.on("/api/close",   HTTP_POST, handleBridgeClose);
  server.on("/api/reset",   HTTP_POST, handleReset);
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println(F("[HTTP] Web server started on port 80"));
}

// ─────────────────────────────────────────────
//  Receive & Process NRF24 Packet
// ─────────────────────────────────────────────
void receivePacket() {
  if (!radio.available()) return;

  uint8_t buf[6];
  radio.read(buf, sizeof(buf));
  packetCount++;
  lastPacketTime = millis();

  // Checksum verification
  if (!verifyChecksum(buf)) {
    Serial.println(F("[RX] Checksum error — packet discarded"));
    errorCount++;
    return;
  }

  // Parse time
  snprintf(lastTimeStr, sizeof(lastTimeStr), "%02d:%02d:%02d",
           buf[0], buf[1], buf[2]);
  memcpy(lastPayload, buf, 6);

  uint8_t status = buf[3];

  Serial.print(F("[RX] Time="));
  Serial.print(lastTimeStr);
  Serial.print(F(" Status=0x"));
  Serial.print(status, HEX);
  Serial.print(F(" Sensors=0b"));
  Serial.println(buf[4], BIN);

  processStatus(status, buf);
}

// ─────────────────────────────────────────────
//  Bridge Control Logic
// ─────────────────────────────────────────────
void processStatus(uint8_t status, uint8_t *payload) {
  if (status == lastStatus) return;  // No change — do nothing
  lastStatus = status;

  switch (status) {
    case STATUS_APPROACHING:
      // Train approaching — CLOSE bridge to allow train passage
      Serial.println(F("[BRIDGE] Train APPROACHING → Closing bridge"));
      closeBridge();
      break;

    case STATUS_DEPARTING:
      // Train departed — OPEN bridge for pedestrians
      Serial.println(F("[BRIDGE] Train DEPARTED → Opening bridge"));
      openBridge();
      break;

    case STATUS_IDLE:
    default:
      Serial.println(F("[BRIDGE] IDLE — No action"));
      break;
  }
}

void openBridge(bool manual) {
  bridgeServo.write(BRIDGE_OPEN_ANGLE);
  bridgeOpen = true;
  lastStatusChange = millis();
  addLog(lastStatus, true, manual);
  Serial.print(F("[SERVO] Bridge OPEN ("));
  Serial.print(BRIDGE_OPEN_ANGLE);
  Serial.println(F("°)"));
}

void closeBridge(bool manual) {
  bridgeServo.write(BRIDGE_CLOSED_ANGLE);
  bridgeOpen = false;
  lastStatusChange = millis();
  addLog(lastStatus, false, manual);
  Serial.print(F("[SERVO] Bridge CLOSED ("));
  Serial.print(BRIDGE_CLOSED_ANGLE);
  Serial.println(F("°)"));
}

void addLog(uint8_t status, bool open, bool manual) {
  memcpy(eventLog[logHead].time, lastTimeStr, 9);
  eventLog[logHead].status       = status;
  eventLog[logHead].bridgeWasOpen = open;
  eventLog[logHead].manual       = manual;
  logHead = (logHead + 1) % LOG_SIZE;
}

bool verifyChecksum(uint8_t *buf) {
  uint8_t cs = 0;
  for (uint8_t i = 0; i < 5; i++) cs ^= buf[i];
  return (cs == buf[5]);
}

// ─────────────────────────────────────────────
//  JSON Status API — /api/status
// ─────────────────────────────────────────────
String buildStatusJSON() {
  String json = "{";
  json += "\"bridge_open\":" + String(bridgeOpen ? "true" : "false") + ",";
  json += "\"manual_override\":" + String(manualOverride ? "true" : "false") + ",";
  json += "\"last_status\":\"0x" + String(lastStatus, HEX) + "\",";
  json += "\"last_packet_time\":\"" + String(lastTimeStr) + "\",";
  json += "\"packets_received\":" + String(packetCount) + ",";
  json += "\"checksum_errors\":" + String(errorCount) + ",";
  json += "\"wifi_rssi\":" + String(WiFi.RSSI()) + ",";
  json += "\"uptime_s\":" + String(millis() / 1000) + ",";
  json += "\"log\":[";

  int idx = logHead;
  for (int i = 0; i < LOG_SIZE; i++) {
    idx = (logHead - 1 - i + LOG_SIZE) % LOG_SIZE;
    if (i > 0) json += ",";
    json += "{\"time\":\"" + String(eventLog[idx].time) + "\",";
    json += "\"open\":" + String(eventLog[idx].bridgeWasOpen ? "true" : "false") + ",";
    json += "\"manual\":" + String(eventLog[idx].manual ? "true" : "false") + "}";
  }
  json += "]}";
  return json;
}

// ─────────────────────────────────────────────
//  HTML Dashboard — /
// ─────────────────────────────────────────────
String buildDashboardHTML() {
  String state   = bridgeOpen ? "OPEN" : "CLOSED";
  String stateClr = bridgeOpen ? "#22c55e" : "#ef4444";
  String statusLabel;
  switch (lastStatus) {
    case STATUS_APPROACHING: statusLabel = "🚆 Train APPROACHING"; break;
    case STATUS_DEPARTING:   statusLabel = "🚆 Train DEPARTED";    break;
    default:                 statusLabel = "💤 IDLE";              break;
  }

  String html = R"rawlit(<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8"/>
  <meta name="viewport" content="width=device-width, initial-scale=1"/>
  <title>NGSRP Bridge Controller</title>
  <style>
    *{box-sizing:border-box;margin:0;padding:0}
    body{font-family:'Segoe UI',sans-serif;background:#0f172a;color:#e2e8f0;min-height:100vh;padding:20px}
    .container{max-width:900px;margin:0 auto}
    h1{text-align:center;font-size:1.6rem;color:#38bdf8;padding:16px 0 4px}
    .subtitle{text-align:center;font-size:0.85rem;color:#64748b;margin-bottom:24px}
    .cards{display:grid;grid-template-columns:repeat(auto-fit,minmax(200px,1fr));gap:16px;margin-bottom:24px}
    .card{background:#1e293b;border-radius:12px;padding:20px;text-align:center;border:1px solid #334155}
    .card h3{font-size:0.75rem;text-transform:uppercase;letter-spacing:1px;color:#94a3b8;margin-bottom:8px}
    .card .val{font-size:1.8rem;font-weight:700}
    .bridge-status{font-size:2rem;font-weight:800}
    .controls{background:#1e293b;border-radius:12px;padding:24px;margin-bottom:24px;border:1px solid #334155}
    .controls h2{font-size:1rem;color:#94a3b8;margin-bottom:16px;text-transform:uppercase;letter-spacing:1px}
    .btn-row{display:flex;gap:12px;flex-wrap:wrap}
    button{padding:12px 28px;border:none;border-radius:8px;font-size:0.95rem;font-weight:600;cursor:pointer;transition:opacity 0.2s}
    button:hover{opacity:0.85}
    .btn-open{background:#22c55e;color:#fff}
    .btn-close{background:#ef4444;color:#fff}
    .btn-auto{background:#3b82f6;color:#fff}
    .btn-reset{background:#64748b;color:#fff}
    .log-table{width:100%;border-collapse:collapse;font-size:0.85rem}
    .log-table th{background:#0f172a;padding:10px;text-align:left;color:#38bdf8;border-bottom:1px solid #334155}
    .log-table td{padding:9px 10px;border-bottom:1px solid #1e293b}
    .log-table tr:nth-child(even){background:#1e293b22}
    .badge{display:inline-block;padding:2px 8px;border-radius:4px;font-size:0.75rem;font-weight:600}
    .badge-open{background:#166534;color:#86efac}
    .badge-closed{background:#7f1d1d;color:#fca5a5}
    .badge-manual{background:#1e3a5f;color:#93c5fd}
    .log-section{background:#1e293b;border-radius:12px;padding:24px;border:1px solid #334155}
    .log-section h2{font-size:1rem;color:#94a3b8;margin-bottom:16px;text-transform:uppercase;letter-spacing:1px}
    .auto-refresh{text-align:center;font-size:0.75rem;color:#475569;margin-top:16px}
    .override-banner{background:#78350f;color:#fef3c7;padding:10px 16px;border-radius:8px;margin-bottom:16px;font-size:0.88rem;display:none}
  </style>
</head>
<body>
<div class="container">
  <h1>🚉 NGSRP Bridge Controller</h1>
  <p class="subtitle">Next-Gen Smart Railway Platform — Real-Time Dashboard</p>

  <div id="override-banner" class="override-banner">
    ⚠️ Manual Override Active — Sensor automation paused
  </div>

  <div class="cards">
    <div class="card">
      <h3>Bridge State</h3>
      <div class="val bridge-status" id="bridge-state" style="color:)rawlit";
  html += stateClr + "\">" + state + "</div></div>";
  html += R"rawlit(
    <div class="card">
      <h3>Sensor Status</h3>
      <div class="val" id="sensor-status" style="font-size:1rem;margin-top:6px">)rawlit";
  html += statusLabel;
  html += R"rawlit(</div></div>
    <div class="card">
      <h3>Last Packet</h3>
      <div class="val" id="last-packet" style="font-size:1.2rem">)rawlit";
  html += String(lastTimeStr);
  html += R"rawlit(</div></div>
    <div class="card">
      <h3>Packets / Errors</h3>
      <div class="val" id="pkt-count" style="font-size:1.2rem">)rawlit";
  html += String(packetCount) + " / " + String(errorCount);
  html += R"rawlit(</div></div>
  </div>

  <div class="controls">
    <h2>Manual Control</h2>
    <div class="btn-row">
      <button class="btn-open"  onclick="bridgeCmd('open')">🔓 Open Bridge</button>
      <button class="btn-close" onclick="bridgeCmd('close')">🔒 Close Bridge</button>
      <button class="btn-auto"  onclick="bridgeCmd('reset')">🤖 Resume Auto</button>
    </div>
  </div>

  <div class="log-section">
    <h2>Operation Log</h2>
    <table class="log-table">
      <thead><tr><th>Time</th><th>Action</th><th>Source</th></tr></thead>
      <tbody id="log-body">)rawlit";

  // Render log rows
  for (int i = 0; i < LOG_SIZE; i++) {
    int idx = (logHead - 1 - i + LOG_SIZE) % LOG_SIZE;
    if (eventLog[idx].time[0] == '\0') continue;
    html += "<tr><td>" + String(eventLog[idx].time) + "</td>";
    html += "<td><span class=\"badge " +
            String(eventLog[idx].bridgeWasOpen ? "badge-open" : "badge-closed") + "\">" +
            String(eventLog[idx].bridgeWasOpen ? "OPENED" : "CLOSED") + "</span></td>";
    html += "<td><span class=\"badge " +
            String(eventLog[idx].manual ? "badge-manual" : "") + "\">" +
            String(eventLog[idx].manual ? "Manual" : "Auto") + "</span></td></tr>";
  }

  html += R"rawlit(
      </tbody>
    </table>
  </div>
  <p class="auto-refresh">Auto-refreshing every 3 seconds</p>
</div>

<script>
function bridgeCmd(action) {
  fetch('/api/' + action, {method:'POST'})
    .then(r => r.json())
    .then(d => { if(d.ok) refreshStatus(); })
    .catch(e => console.error(e));
}

function refreshStatus() {
  fetch('/api/status')
    .then(r => r.json())
    .then(d => {
      const el = id => document.getElementById(id);
      el('bridge-state').textContent = d.bridge_open ? 'OPEN' : 'CLOSED';
      el('bridge-state').style.color = d.bridge_open ? '#22c55e' : '#ef4444';
      el('last-packet').textContent  = d.last_packet_time;
      el('pkt-count').textContent    = d.packets_received + ' / ' + d.checksum_errors;
      const banner = el('override-banner');
      banner.style.display = d.manual_override ? 'block' : 'none';

      let statusMap = {'0x1':'🚆 Train APPROACHING','0x10':'🚆 Train DEPARTED'};
      el('sensor-status').textContent = statusMap[d.last_status] || '💤 IDLE';

      // Rebuild log
      let rows = '';
      for (const e of d.log) {
        if (!e.time || e.time === '00:00:00') continue;
        rows += '<tr><td>' + e.time + '</td>';
        rows += '<td><span class="badge ' + (e.open ? 'badge-open' : 'badge-closed') + '">'
                + (e.open ? 'OPENED' : 'CLOSED') + '</span></td>';
        rows += '<td><span class="badge ' + (e.manual ? 'badge-manual' : '') + '">'
                + (e.manual ? 'Manual' : 'Auto') + '</span></td></tr>';
      }
      el('log-body').innerHTML = rows;
    })
    .catch(e => console.error('Status fetch error:', e));
}

setInterval(refreshStatus, 3000);
</script>
</body>
</html>)rawlit";

  return html;
}

// ─────────────────────────────────────────────
//  HTTP Route Handlers
// ─────────────────────────────────────────────
void handleRoot() {
  server.send(200, "text/html", buildDashboardHTML());
}

void handleStatus() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", buildStatusJSON());
}

void handleBridgeOpen() {
  manualOverride = true;
  openBridge(true);
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", "{\"ok\":true,\"action\":\"opened\",\"manual\":true}");
}

void handleBridgeClose() {
  manualOverride = true;
  closeBridge(true);
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", "{\"ok\":true,\"action\":\"closed\",\"manual\":true}");
}

void handleReset() {
  manualOverride = false;
  lastStatus = STATUS_IDLE;
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", "{\"ok\":true,\"action\":\"auto_resumed\"}");
  Serial.println(F("[WEB] Manual override cleared — Auto mode restored"));
}

void handleNotFound() {
  server.send(404, "text/plain", "NGSRP: Route not found");
}
