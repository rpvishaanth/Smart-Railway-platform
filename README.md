# Smart-Railway-platform

<p align="center">
  <img src="https://img.shields.io/badge/Platform-Arduino%20Uno%20%7C%20ESP8266-blue?style=for-the-badge&logo=arduino"/>
  <img src="https://img.shields.io/badge/RF-NRF24L01-orange?style=for-the-badge"/>
  <img src="https://img.shields.io/badge/Protocol-SPI%20%7C%20WiFi-green?style=for-the-badge"/>
  <img src="https://img.shields.io/badge/License-MIT-lightgrey?style=for-the-badge"/>
</p>

---

## 📖 Overview

The **Next-Gen Smart Railway Platform (NGSRP)** is an IoT-based system designed to enhance accessibility and safety at railway stations. It automates a **mobile bridge** (a retractable pedestrian walkway between platforms) by detecting train arrivals and departures through sensors, and controlling the bridge remotely over NRF24L01 wireless RF communication.

The system is especially beneficial for **elderly passengers and persons with disabilities (PWD)**, eliminating the need for overhead footbridges and staircases.

> 

---

## 🗺️ System Architecture

```
┌──────────────────────────────────────────┐
│           PRIMARY SENSOR NODE            │
│           (Arduino Uno — TX)             │
│                                          │
│  Hall Effect Sensor 1 ──┐               │
│  Hall Effect Sensor 2 ──┼── Controller ─┼──► NRF24L01 TX
│  Vibration Sensor ──────┘               │
└──────────────────────────────────────────┘
                   │ RF (2.4 GHz, Ch 76)
                   ▼
┌──────────────────────────────────────────┐
│         BRIDGE CONTROLLER NODE           │
│         (ESP8266 NodeMCU — RX)           │
│                                          │
│  NRF24L01 RX ──► Controller             │
│                      │                  │
│                      ├──► Servo Motor   │
│                      │    (Bridge)      │
│                      └──► WiFi ──► Web  │
│                               Dashboard │
└──────────────────────────────────────────┘
```

---

## ✨ Features

- 🔍 **Dual Hall Effect Sensors** — detect train direction (approaching vs. departing) based on trigger order
- 📳 **Vibration Sensor** — confirms physical track vibration for multi-sensor reliability
- 📡 **NRF24L01 Wireless RF** — 2.4 GHz reliable long-range communication between nodes
- 🔒 **Payload Checksum** — XOR-based integrity check on every transmitted packet
- ⚙️ **Servo Motor Control** — 0° closed (train passage) / 90° open (pedestrian crossing)
- 🌐 **Live Web Dashboard** — hosted on ESP8266's IP; view bridge state, sensor status, and operation log
- 🕹️ **Manual Override** — open/close the bridge from the web UI and resume auto mode
- 📋 **Operation Event Log** — last 10 bridge events displayed in the dashboard
- 📶 **WiFi AP Fallback** — if home WiFi fails, ESP8266 creates its own hotspot (`NGSRP-Bridge`)
- 🕐 **Software Clock** — set station time via Serial command (`TIME:HH:MM:SS`)

---

## 🛒 Hardware Requirements

### Common (both nodes)
| Component | Qty | Notes |
|---|---|---|
| NRF24L01+ Module | 2 | 2.4 GHz transceiver |
| 10µF electrolytic capacitor | 2 | Across VCC/GND of each NRF module |
| Breadboard + jumper wires | — | — |

### Node 1 — Sensor Transmitter
| Component | Qty | Notes |
|---|---|---|
| Arduino Uno (or Nano) | 1 | |
| Hall Effect Sensor (A3144 or SS49E) | 2 | For wheel/magnet detection |
| Vibration Sensor (SW-420 module) | 1 | Analog output |
| 9V battery or USB power | 1 | |

### Node 2 — Bridge Controller
| Component | Qty | Notes |
|---|---|---|
| ESP8266 NodeMCU v1.0 (ESP-12E) | 1 | |
| SG90 / MG995 Servo Motor | 1 | Drives the mobile bridge |
| 5V external power supply | 1 | For servo (NodeMCU 3.3V insufficient) |

---

## 🔌 Wiring Diagrams

### NRF24L01 → Arduino Uno (Transmitter)

| NRF24L01 Pin | Arduino Uno Pin |
|---|---|
| VCC | 3.3 V |
| GND | GND |
| CE | D9 |
| CSN | D10 |
| SCK | D13 |
| MOSI | D11 |
| MISO | D12 |

### Sensors → Arduino Uno

| Component | Arduino Pin | Type |
|---|---|---|
| Hall Sensor 1 (OUT) | D2 | Digital INPUT_PULLUP |
| Hall Sensor 2 (OUT) | D3 | Digital INPUT_PULLUP |
| Vibration Sensor (AO) | A0 | Analog |
| Hall Sensor VCC | 5 V | Power |
| Vibration Sensor VCC | 5 V | Power |
| All GND | GND | Ground |

### NRF24L01 → NodeMCU ESP8266 (Receiver)

| NRF24L01 Pin | NodeMCU Pin | GPIO |
|---|---|---|
| VCC | 3.3 V | — |
| GND | GND | — |
| CE | D4 | GPIO 2 |
| CSN | D8 | GPIO 15 |
| SCK | D5 | GPIO 14 |
| MOSI | D7 | GPIO 13 |
| MISO | D6 | GPIO 12 |

### Servo → NodeMCU ESP8266

| Servo Wire | Connection |
|---|---|
| Signal (Orange) | D3 (GPIO 0) |
| VCC (Red) | External 5V supply |
| GND (Brown) | GND (common with NodeMCU) |

> ⚠️ **Always use an external 5V supply for the servo.** Powering it from NodeMCU's 3.3 V or onboard 5 V regulator will cause brownouts and WiFi disconnections.

---

## 💾 Software Setup

### 1. Arduino IDE Setup

Install the **Arduino IDE** (v1.8.x or 2.x): https://www.arduino.cc/en/software

### 2. Add ESP8266 Board Support

In Arduino IDE → **File → Preferences → Additional Boards Manager URLs**, add:

```
http://arduino.esp8266.com/stable/package_esp8266com_index.json
```

Then go to **Tools → Board → Boards Manager** and install **esp8266 by ESP8266 Community**.

Board setting for receiver: `NodeMCU 1.0 (ESP-12E Module)`

### 3. Install Libraries

Open **Tools → Manage Libraries** and install:

| Library | Author | Used By |
|---|---|---|
| RF24 | TMRh20 / Avamander | Both nodes |
| Servo | Built-in | ESP8266 receiver |

ESP8266WiFi and ESP8266WebServer are bundled with the ESP8266 board package — no separate install needed.

---

## 🚀 Quick Start

### Step 1 — Configure WiFi (ESP8266 only)

Open `esp8266_receiver/esp8266_receiver.ino` and edit lines 24–25:

```cpp
const char* WIFI_SSID     = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";
```

### Step 2 — Upload Transmitter Code

1. Connect Arduino Uno via USB
2. Open `arduino_transmitter/arduino_transmitter.ino`
3. Select **Board: Arduino Uno** and correct COM port
4. Click **Upload**

### Step 3 — Upload Receiver Code

1. Connect NodeMCU via USB
2. Open `esp8266_receiver/esp8266_receiver.ino`
3. Select **Board: NodeMCU 1.0 (ESP-12E Module)** and correct COM port
4. Set **Upload Speed: 115200**
5. Click **Upload**

### Step 4 — Power Up & Connect

1. Power both nodes (USB or battery)
2. Open Arduino Uno Serial Monitor (9600 baud) to see sensor events
3. Open NodeMCU Serial Monitor (115200 baud) — it will print the assigned IP address
4. Open a browser and navigate to `http://<ESP8266-IP>/`

---

## 🌐 Web Dashboard

The ESP8266 hosts a real-time web dashboard at its local IP address.

```
http://192.168.x.x/        → Main dashboard
http://192.168.x.x/api/status  → JSON status API
```

### REST API

| Endpoint | Method | Description |
|---|---|---|
| `/api/status` | GET | Returns full JSON system status |
| `/api/open` | POST | Manually open bridge (enables override) |
| `/api/close` | POST | Manually close bridge (enables override) |
| `/api/reset` | POST | Resume automatic sensor-driven control |

**Example status response:**
```json
{
  "bridge_open": false,
  "manual_override": false,
  "last_status": "0x1",
  "last_packet_time": "14:32:07",
  "packets_received": 42,
  "checksum_errors": 0,
  "wifi_rssi": -61,
  "uptime_s": 3720,
  "log": [...]
}
```

---

## 📡 RF Payload Format

Every packet is exactly **6 bytes**:

| Byte | Field | Description |
|---|---|---|
| 0 | Hour | 0–23 (24-hour clock) |
| 1 | Minute | 0–59 |
| 2 | Second | 0–59 |
| 3 | Status | `0x01` = Approaching · `0x10` = Departing · `0x00` = Idle |
| 4 | Sensor Flags | Bit 0 = Hall1 · Bit 1 = Hall2 · Bit 2 = Vibration |
| 5 | Checksum | XOR of bytes 0–4 |

RF settings: Channel 76 (2.476 GHz) · 250 kbps · CRC-16 · PA High

---

## 🧠 Train Direction Detection

The system infers train direction from the **order** in which the two Hall sensors are triggered:

```
Direction of travel →

     [Hall 1]           [Hall 2]        → APPROACHING
       │                   │
   triggers first      triggers second

     [Hall 2]           [Hall 1]        → DEPARTING
       │                   │
   triggers first      triggers second
```

A vibration sensor reading above the threshold (`VIBRATION_THRESHOLD = 450`) is also required to confirm the detection, preventing false triggers from stray magnetic fields.

---

## ⚙️ Configurable Parameters

### Arduino Transmitter (`arduino_transmitter.ino`)

| Constant | Default | Description |
|---|---|---|
| `VIBRATION_THRESHOLD` | 450 | ADC value above which vibration is detected (0–1023) |
| `DEBOUNCE_MS` | 50 | Hall sensor debounce time in ms |
| `TRAIN_CONFIRM_WINDOW_MS` | 3000 | Max time between H1/H2 triggers to count as same train |
| `IDLE_TIMEOUT_MS` | 8000 | Time with no detection before returning to IDLE |
| `TRANSMIT_INTERVAL_MS` | 500 | Retransmit interval while active |

### ESP8266 Receiver (`esp8266_receiver.ino`)

| Constant | Default | Description |
|---|---|---|
| `BRIDGE_OPEN_ANGLE` | 90 | Servo angle when bridge is open (degrees) |
| `BRIDGE_CLOSED_ANGLE` | 0 | Servo angle when bridge is closed (degrees) |
| Packet timeout | 30 s | Auto-close bridge if no packet received |

---

## 🔧 Serial Commands (Transmitter)

Connect to the Arduino Uno at **9600 baud** and send:

| Command | Description |
|---|---|
| `TIME:HH:MM:SS` | Set the software clock (e.g. `TIME:14:30:00`) |
| `STATUS` | Print current train status code |
| `RESET` | Force status back to IDLE |

---

## 📂 Repository Structure

```
smart-railway-platform/
│
├── system_block_diagram.png          # Full system block diagram
│
├── arduino_transmitter/
│   └── arduino_transmitter.ino       # Sensor node — Hall + Vibration + NRF TX
│
├── esp8266_receiver/
│   └── esp8266_receiver.ino          # Bridge controller — NRF RX + Servo + Web server
│
└── README.md                         # This file
```

---

## 🚧 Troubleshooting

| Problem | Likely Cause | Fix |
|---|---|---|
| NRF module not found | Wiring error or insufficient power | Add 10µF cap across VCC/GND; double-check SPI pins |
| Bridge doesn't move | Servo underpowered | Use external 5V supply for servo |
| WiFi not connecting | Wrong SSID/password | Check credentials; NodeMCU will fall back to AP mode |
| Dashboard not loading | Wrong IP | Check NodeMCU Serial Monitor for printed IP |
| False train detections | Low vibration threshold | Increase `VIBRATION_THRESHOLD` in transmitter code |
| Checksum errors | RF interference | Try a different channel (edit `radio.setChannel()` on both nodes — keep them matching) |
| Servo jitters | Noise on signal line | Add 100Ω resistor in series on signal wire |

---

## 📊 Power Consumption (Theoretical)

| Component | Voltage | Current | Power |
|---|---|---|---|
| Hall Effect Sensor × 2 | 3.3–5 V | 19–24 mA | 0.03–0.05 W each |
| Vibration Sensor | 5 V | 12–15 mA | 0.06–0.08 W |
| NRF24L01 (TX) | 3.3–5 V | 117–122 mA | 0.38–0.43 W |
| Arduino Uno (MCU) | 7–9 V | 18–25 mA | 0.19–0.22 W |
| **Total (Primary Node)** | | | **~0.66–0.78 W** |

The low power consumption makes solar panel + battery backup a viable power source for the sensor node installed trackside.

---

## 🔭 Future Scope

- **Cloud Integration** — Push data to Adafruit IO or Firebase for remote monitoring and historical logging
- **Railway API Integration** — Connect to Indian Railways API for real-time train schedule cross-referencing
- **Multiple Bridge Support** — Expand to 3+ bridge controllers, each listening on a unique pipe address
- **AI Prediction** — Train an ML model on historical bridge operation data to predict opening/closing times
- **LCD Display** — Add a 16×2 LCD to the ESP8266 node showing bridge status and timing locally
- **LoRa Upgrade** — Replace NRF24L01 with LoRa (SX1276) for kilometre-range sensor placement
- **OTA Updates** — Enable over-the-air firmware updates via Arduino OTA for ESP8266


## 📄 License

This project is licensed under the **MIT License** — see the [LICENSE](LICENSE) file for details.

---

## 📚 References

- Daniel D, A., Vishaanth, R. P., & Saravanan, T. K. (2024). *Smart Railway Platform with Train Arrival Detection.* International Journal of Microsystems and IoT, 2(3), 632–639. https://doi.org/10.5281/zenodo.10842058
- Velayutham, R., Sangeethavani, T., & Sundaralakshmi, K. (2017). *Controlling railway gates using smart phones by tracking trains with GPS.* IEEE ICCPCT.
- Jayasinghe, U., & Kathriarachchi, P. (2021). *Real-time Train Tracking System in Sri Lanka.* IEEE ICIAfS.
- Róbert, M. et al. (2022). *Internet Based Control of a Servo Motor with a Sliding Mode Based Observer.* IEEE PEMC.

---

<p align="center">Made with ❤️ for safer, more accessible railway stations · Sri Sai Ram Engineering College</p>
