alopt# Project Title
JBD-Sungrow ESP32 Bridge (Pylontech CAN Emulator with Web UI)

---

## Goal
Create a fully working ESP32-based firmware system to interface a **JBD BMS (UP16S010, RS485/CAN)** with a **Sungrow SH5K-30 inverter** using **Pylontech CAN emulation**. The project must:

- Read data from JBD BMS over RS485 via MAX485 (HW-97 board)
- Send CAN messages using Pylontech-compatible format via MCP2515+TJA1050 (QYF-983 board, 16 MHz crystal)
- Run on an **ESP32 DevKit V1 (WROOM)** using UART + SPI
- Expose a **mobile-friendly Web UI** over WiFi (Station mode with fallback AP/captive portal)
- Support **OTA firmware updates** via the browser
- Use **SPIFFS** or **LittleFS** to persist config (battery capacity, module count, CAN send rate)
- Be fully flashable via PlatformIO, with structured folders

---

## Boards & Interfaces

- **ESP32 DevKit V1**  
  - 3.3 V logic  
  - 5 V supply rail available on VIN pin

- **HW-97 MAX485 board** (for RS485 from JBD)  
  - Powered via 5 V pin from ESP32  
  - Connected to JBD RS485 RJ45:  
    - Pin 1 = A (orange/white)  
    - Pin 2 = B (orange)  
    - GND from pin 3 or 6

- **QYF-983 MCP2515 + TJA1050 CAN board**  
  - Powered by ESP32 5 V pin  
  - 16 MHz crystal  
  - Connects to CAN-H / CAN-L on Sungrow RJ45:  
    - Pin 4 = CAN-H (blue)  
    - Pin 5 = CAN-L (blue/white)  
    - GND = RJ45 pin 2 or 8

---

## Pin Assignments

- UART2 (JBD BMS):
  - `RX` = GPIO 16
  - `TX` = GPIO 17
  - `RE/DE` control = GPIO 25 (LOW to receive)

- MCP2515 SPI (CAN output):
  - `CS` = GPIO 5  
  - `INT` = GPIO 4  
  - `SCK` = GPIO 18  
  - `MOSI` = GPIO 23  
  - `MISO` = GPIO 19

---

## CAN Details

- CAN Bus speed: **500 kbps**
- Format: Standard Pylontech LV messages
- Main CAN frame: `0x4210` (contains SoC, voltage, current, temperature)
- Message frequency: Configurable (default 1 frame/sec)

---

## Web UI Requirements

- Mobile-friendly, light/dark mode
- View live:
  - Voltage, Current, SoC, Temp
  - Alarm/fault status
  - CAN send diagnostics
- Editable config:
  - Battery capacity (kWh)
  - Module count (e.g. 7 for 17.4 kWh)
  - CAN message interval
  - Alarm toggles
  - CAN IDs (advanced)
- Captive portal fallback if WiFi not configured
- OTA firmware upload from browser
- Config stored in JSON (SPIFFS)

---

## Project Structure (PlatformIO)

```
JBD_Sungrow_Bridge/
├── platformio.ini
├── src/
│   └── main.cpp
├── data/
│   ├── index.html     ← Web UI
│   ├── config.json    ← Persistent settings
├── include/           ← (optional for web/lib code)
├── README.md
```

---

## Libraries Required
- `WiFiManager` (or native captive portal logic)
- `ESPAsyncWebServer` (or WebServer for simplicity)
- `SPIFFS` (for config and web UI)
- `MCP_CAN_lib` for MCP2515
- `ArduinoJson` for config handling

---

## Functionality Summary

On startup:
- Connects to saved WiFi (Station mode)
- If not configured: starts AP (`JBDBridge-Setup` / `setup1234`) with captive portal
- Polls JBD BMS every 1s using `0xDD A5 03 00 FF 77` command
- Parses voltage, current, SoC, temps from BMS
- Sends Pylontech-compatible CAN message via MCP2515 (`0x4210`)
- Serves a Web UI at `/` with stats, editable settings, and OTA upload
- All settings saved to `/config.json` on SPIFFS

---

## Next Step
Generate all files to match this structure: `main.cpp`, `platformio.ini`, `index.html`, `config.json`, and scaffolding for OTA + SPIFFS mounting. Ensure real CAN output logic and BMS polling are implemented. The system must also:
- Parse the charge/discharge MOSFET state from the JBD response (byte 22)
- Parse and decode the fault flags (byte 23) into specific BMS errors
- Reflect these states in the CAN status messages:
  - If discharge MOSFET is off, report 0A discharge allowed
  - If charge MOSFET is off, report 0A charge allowed
  - If any protection fault is active, raise alarm flags in the CAN protocol
- Optionally add a "read-only MOSFET override mode" in the web UI for testing

> Note: the JBD BMS cannot be commanded to enable/disable MOSFETs remotely — the inverter cannot directly control it. The bridge only reflects BMS state to Sungrow. 