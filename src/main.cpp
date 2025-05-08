#include <Arduino.h>
#include <SPI.h>
#include <mcp_can.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <Update.h>
#include <esp_task_wdt.h>

// -----------------------------------------------------------------------------
// Pin‑mapping & hardware constants
// -----------------------------------------------------------------------------
// On most ESP32 dev boards, GPIO2 is connected to an internal LED
// Some boards use GPIO5, others use GPIO22
#define LED_BUILTIN        22      // use a dedicated status LED
#define CAN_CS             5       // MCP2515 CS
#define VSPI_MISO          19
#define VSPI_MOSI          23
#define VSPI_SCLK          18
#define POWER_MONITOR_PIN  32      // ADC1_CH4 (GPIO32)
#define CAN_INT            4       // MCP2515 INT output
#define CAN_RESET          25      // MCP2515 RESET pin

// RS‑485 transceiver (MAX3485) pins
#define RS485_RX           16
#define RS485_TX           17
#define RS485_DE           26      // driver‑enable (was 25, moved to avoid clash with CAN_RESET)

// -----------------------------------------------------------------------------
// Timing / behaviour constants
// -----------------------------------------------------------------------------
constexpr uint16_t LED_BLINK_INTERVAL = 250;   // ms
constexpr uint16_t RS485_TX_SETTLE_US = 104;   // µs (one char @ 9600)

#define JBD_BAUD          9600
#define CAN_SPEED         MCP_16MHZ
#define CAN_RATE          CAN_500KBPS
#define CAN_MODE          MCP_NORMAL
#define SPI_FREQUENCY     4000000     // 4 MHz
#define JBD_POLL_INTERVAL 1000        // ms

#define CONFIG_FILE       "/config.json"
#define LOG_BUFFER_SIZE   1000
#define WDT_TIMEOUT       10          // seconds
#define MAX_RECOVERY_ATTEMPTS 3
#define RECOVERY_DELAY    1000        // ms

#define MODE_MASK         0xE0        // Bits 7‑5 of CANSTAT

// -----------------------------------------------------------------------------
// Extra MCP2515 register definitions not provided by the library
// -----------------------------------------------------------------------------
#ifndef MCP_CANINTF
  #define MCP_CANINTF   0x2C
#endif
#ifndef MCP_EFLG
  #define MCP_EFLG      0x2D   // Error Flag register
#endif
#ifndef MCP_TEC
  #define MCP_TEC       0x1C   // TX error counter
#endif
#ifndef MCP_REC
  #define MCP_REC       0x1D   // RX error counter
#endif
#ifndef MCP_CANSTAT
  #define MCP_CANSTAT   0x0E   // Status register (mode bits)
#endif
#ifndef MCP_TXB0CTRL
  #define MCP_TXB0CTRL  0x30   // TX buffer 0 control
#endif
#ifndef MCP_TXB1CTRL
  #define MCP_TXB1CTRL  0x40   // TX buffer-1 control
#endif
#ifndef MCP_TXB2CTRL
  #define MCP_TXB2CTRL  0x50   // TX buffer-2 control
#endif
#ifndef MCP_RX0IF
  #define MCP_RX0IF     0x01
#endif
#ifndef MCP_RX1IF
  #define MCP_RX1IF     0x02
#endif
#ifndef MCP_TX0IF
  #define MCP_TX0IF     0x04
#endif
#ifndef MCP_TX1IF
  #define MCP_TX1IF     0x08
#endif
#ifndef MCP_TX2IF
  #define MCP_TX2IF     0x10
#endif
#ifndef MCP_ERRIF
  #define MCP_ERRIF     0x20
#endif
#ifndef MCP_WAKIF
  #define MCP_WAKIF     0x40
#endif
#ifndef MCP_MERRF
  #define MCP_MERRF     0x80
#endif

// -----------------------------------------------------------------------------
//  MCP2515 mode definitions
// -----------------------------------------------------------------------------
#define MCP_CONFIG    0x80
#define MCP_NORMAL    0x00
#define MCP_SLEEP     0x20
#define MCP_LOOPBACK  0x40
#define MCP_LISTEN    0x60

// -----------------------------------------------------------------------------
//  MCP2515 command bytes (for low‑level helper functions)
// -----------------------------------------------------------------------------
#define MCP_RESET        0xC0
#define MCP_READ         0x03
#define MCP_WRITE        0x02

// -----------------------------------------------------------------------------
// 500 kbit/s timing (@ 16 MHz crystal)
// -----------------------------------------------------------------------------
#define CNF1_500KBPS 0x00      // SJW = 1 TQ, BRP = 0
#define CNF2_500KBPS 0x90      // BTLMODE=1, PHSEG1 = 2 TQ, PRSEG = 2 TQ
#define CNF3_500KBPS 0x02      // PHSEG2 = 2 TQ

// 250 kbit/s – more relaxed
#define CNF1_250KBPS 0x03
#define CNF2_250KBPS 0x90
#define CNF3_250KBPS 0x02

// -----------------------------------------------------------------------------
//  Diagnostics – loop‑back test frame
// -----------------------------------------------------------------------------
#define TEST_MSG_ID  0x123

// -----------------------------------------------------------------------------
// Forward declarations & utility prototypes
// -----------------------------------------------------------------------------
static inline uint8_t readRegister(uint8_t addr);
static inline void    writeRegister(uint8_t addr, uint8_t val);
void   hardwareReset();
void   checkBusStatus();

void   addLog(const String &msg);

// More high‑level prototypes live further down (just to unclutter header).

// -----------------------------------------------------------------------------
// Global singletons & state structs
// -----------------------------------------------------------------------------
MCP_CAN           CAN(CAN_CS);
AsyncWebServer    server(80);
WiFiManager       wifiManager;

struct Metrics {
  uint32_t txErrors = 0;
  uint32_t rxErrors = 0;
  int16_t  wifiRSSI = 0;
  float    supplyVoltage = 0.0f;
  float    rawVoltage = 0.0f;    // Cache raw ADC reading
  uint32_t lastUpdate = 0;
} metrics;

enum BMSStatus : uint8_t { BMS_OK, BMS_TIMEOUT, BMS_ERROR };
volatile BMSStatus lastBMSStatus = BMS_TIMEOUT;

volatile bool canActive      = false;
volatile bool canError       = false;
volatile bool canInitialized = false;
unsigned long lastCanSend    = 0;

struct RecoveryState {
  uint8_t  canRecoveryAttempts  = 0;
  uint8_t  wifiRecoveryAttempts = 0;
  uint32_t lastCanRecovery      = 0;
  uint32_t lastWifiRecovery     = 0;
  bool     inRecovery           = false;
} recoveryState;

// Add lastLedBlink with other global variables
unsigned long lastLedBlink = 0;

// Ring‑buffer for human log lines ------------------------------------------------
String logBuffer[LOG_BUFFER_SIZE];
int    logIndex = 0;

// -----------------------------------------------------------------------------
//  Configuration persists in LittleFS
// -----------------------------------------------------------------------------
struct Config {
  float   batteryCapacity;      // kWh
  uint8_t moduleCount;          // # JBD modules
  uint16_t canInterval;         // ms between outbound frames
  bool    alarmEnabled;
  uint32_t canId;
  float   voltageDividerRatio;  // ADC scaling
  String  apName;
  String  apPassword;
  // AP network configuration
  uint8_t apIP[4];             // Default to non-routable range
  uint8_t apGateway[4];
  uint8_t apSubnet[4];
};

// Initialize config with default values
Config config = {
  17.4f,              // batteryCapacity
  7,                  // moduleCount
  1000,               // canInterval
  true,               // alarmEnabled
  0x4210,             // canId
  10.57f,             // voltageDividerRatio
  "",                 // apName
  "",                 // apPassword
  {192, 168, 4, 1},   // apIP
  {192, 168, 4, 1},   // apGateway
  {255, 255, 255, 0}  // apSubnet
};

struct BMSData {
  float   voltage      = NAN;
  float   current      = NAN;
  uint8_t soc          = 255; // 255 = invalid
  int8_t  temperature  = -128; // -128 = invalid
  float   power        = NAN;
  uint32_t lastUpdate  = 0;
  bool    chargeEnabled    = true;
  bool    dischargeEnabled = true;
  uint8_t faultFlags       = 0;
} bmsData;

// -----------------------------------------------------------------------------
// Low‑level helpers ------------------------------------------------------------
// -----------------------------------------------------------------------------
static inline uint8_t readRegister(uint8_t addr) {
  digitalWrite(CAN_CS, LOW);
  SPI.transfer(MCP_READ);
  SPI.transfer(addr);
  uint8_t val = SPI.transfer(0x00);
  digitalWrite(CAN_CS, HIGH);
  return val;
}

static inline void writeRegister(uint8_t addr, uint8_t val) {
  digitalWrite(CAN_CS, LOW);
  SPI.transfer(MCP_WRITE);
  SPI.transfer(addr);
  SPI.transfer(val);
  digitalWrite(CAN_CS, HIGH);
}

// Board‑level helpers ----------------------------------------------------------
void hardwareReset() {
  pinMode(CAN_RESET, OUTPUT);
  digitalWrite(CAN_RESET, LOW);
  delay(100);
  digitalWrite(CAN_RESET, HIGH);
}

void rs485TxBegin() {
  digitalWrite(RS485_DE, HIGH);
  delayMicroseconds(RS485_TX_SETTLE_US);
}

void rs485TxEnd() {
  delayMicroseconds(RS485_TX_SETTLE_US);
  digitalWrite(RS485_DE, LOW);
}

// ----------------------------------------------------------------------------
// Logging convenience --------------------------------------------------------
void addLog(const String &msg) {
  logBuffer[logIndex] = String(millis()) + F(": ") + msg;
    logIndex = (logIndex + 1) % LOG_BUFFER_SIZE;
    Serial.println(msg);
}

// ----------------------------------------------------------------------------
//  Function prototypes implemented later (kept here for quick reference)
// ----------------------------------------------------------------------------
void setupWiFi();
void setupWebServer();
void loadConfig();
void saveConfig();
void setupCAN();
bool setupCANController();
void sendCANMessage();
byte sendCANMessageWithParams(unsigned long id, byte *data, byte len);
void monitorMessages();
bool sendTestMessage();
bool attemptCANRecovery();
bool attemptWiFiRecovery();
float readPowerVoltage();

// =============================================================================
//  High‑level helpers & API formatting
// =============================================================================
String getSystemStatus() {
  StaticJsonDocument<384> doc;
  if (isnan(bmsData.voltage)) doc["voltage"].set(nullptr); else doc["voltage"] = bmsData.voltage;
  if (isnan(bmsData.current)) doc["current"].set(nullptr); else doc["current"] = bmsData.current;
  if (bmsData.soc == 255) doc["soc"].set(nullptr); else doc["soc"] = bmsData.soc;
  if (bmsData.temperature == -128) doc["temperature"].set(nullptr); else doc["temperature"] = bmsData.temperature;
  doc["chargeEnabled"]  = bmsData.chargeEnabled;
  doc["dischargeEnabled"] = bmsData.dischargeEnabled;
  doc["wifiConnected"]  = (WiFi.status() == WL_CONNECTED);
  doc["canTEC"]         = readRegister(MCP_TEC);
  doc["canREC"]         = readRegister(MCP_REC);
  String out;
  serializeJson(doc, out);
  return out;
}

String getSystemConfig() {
  StaticJsonDocument<384> doc;
  doc["batteryCapacity"]     = config.batteryCapacity;
  doc["moduleCount"]         = config.moduleCount;
  doc["canInterval"]         = config.canInterval;
  doc["alarmEnabled"]        = config.alarmEnabled;
  doc["canId"]               = config.canId;
  doc["voltageDividerRatio"] = config.voltageDividerRatio;
  String out;
  serializeJson(doc, out);
  return out;
}

// =============================================================================
//  Wi‑Fi recovery & CAN recovery helpers
// =============================================================================
bool attemptWiFiRecovery() {
  if (recoveryState.wifiRecoveryAttempts >= MAX_RECOVERY_ATTEMPTS) {
    addLog(F("Max WiFi recovery attempts reached – rebooting"));
    ESP.restart();
    return false; // unreachable
  }
  addLog(F("Attempting WiFi recovery…"));
  recoveryState.wifiRecoveryAttempts++;
  recoveryState.lastWifiRecovery = millis();

  WiFi.disconnect(true);
  delay(1000);
  esp_task_wdt_reset();

  wifiManager.resetSettings();
  delay(1000);
  esp_task_wdt_reset();

  setupWiFi();
  if (WiFi.status() == WL_CONNECTED) {
    addLog(F("WiFi recovery successful"));
    recoveryState.wifiRecoveryAttempts = 0;
    return true;
  }
  addLog(F("WiFi recovery failed"));
  return false;
}

#define MAX_RECOVERY_DELAY 30000  // Maximum recovery delay (30 seconds)

bool attemptCANRecovery() {
  if (recoveryState.canRecoveryAttempts >= MAX_RECOVERY_ATTEMPTS) {
    addLog(F("Max CAN recovery attempts reached – rebooting"));
    ESP.restart();
    return false; // unreachable
  }
  addLog(F("Attempting CAN recovery…"));
  recoveryState.canRecoveryAttempts++;
  
  // Calculate exponential backoff delay, capped at MAX_RECOVERY_DELAY
  uint32_t backoffDelay = min(RECOVERY_DELAY << (recoveryState.canRecoveryAttempts - 1), MAX_RECOVERY_DELAY);
  delay(backoffDelay);
  esp_task_wdt_reset();
  
  // Update last recovery time after the delay
  recoveryState.lastCanRecovery = millis();

  // SPI is already initialized in setup(), no need to reinitialize
  pinMode(CAN_CS, OUTPUT);
  digitalWrite(CAN_CS, HIGH);

  if (setupCANController()) {
    addLog(F("CAN recovery successful"));
    recoveryState.canRecoveryAttempts = 0;
    canError = false;
    return true;
  }
  addLog(F("CAN recovery failed"));
  return false;
}

// =============================================================================
//  Configuration persistence helpers
// =============================================================================
void saveConfig() {
  // JSON size calculation:
  // - Base fields: ~200 bytes
  // - AP network arrays (3 arrays × 4 bytes × 2 chars + overhead): ~40 bytes
  // - Current scaling (0.1A steps): no impact on size
  // - Total with margin: 640 bytes is sufficient
  // To verify size: use measureJson() in unit test:
  //   StaticJsonDocument<640> doc;
  //   ... populate doc ...
  //   String out;
  //   serializeJson(doc, out);
  //   Serial.printf("JSON size: %d bytes\n", measureJson(doc));
  StaticJsonDocument<640> doc;  // Increased from 512 to accommodate arrays
  doc["batteryCapacity"]     = config.batteryCapacity;
  doc["moduleCount"]         = config.moduleCount;
  doc["canInterval"]         = config.canInterval;
  doc["alarmEnabled"]        = config.alarmEnabled;
  doc["canId"]               = config.canId;
  doc["voltageDividerRatio"] = config.voltageDividerRatio;
  doc["apName"]              = config.apName;
  doc["apPassword"]          = config.apPassword;
  
  // Save AP network configuration
  JsonArray apIP = doc.createNestedArray("apIP");
  JsonArray apGateway = doc.createNestedArray("apGateway");
  JsonArray apSubnet = doc.createNestedArray("apSubnet");
  for (int i = 0; i < 4; i++) {
    apIP.add(config.apIP[i]);
    apGateway.add(config.apGateway[i]);
    apSubnet.add(config.apSubnet[i]);
  }

  if (File f = LittleFS.open(CONFIG_FILE, "w")) {
    serializeJson(doc, f);
    f.close();
    addLog(F("Config saved"));
  } else {
    addLog(F("! Failed to open config for writing"));
  }
}

void loadConfig() {
  if (!LittleFS.exists(CONFIG_FILE)) {
    addLog(F("Config file absent – using defaults"));
    return;
  }
  if (File f = LittleFS.open(CONFIG_FILE, "r")) {
    StaticJsonDocument<640> doc;  // Increased from 512
    if (deserializeJson(doc, f) == DeserializationError::Ok) {
      config.batteryCapacity     = doc["batteryCapacity"]     | config.batteryCapacity;
      config.moduleCount         = doc["moduleCount"]         | config.moduleCount;
      config.canInterval         = doc["canInterval"]         | config.canInterval;
      config.alarmEnabled        = doc["alarmEnabled"]        | config.alarmEnabled;
      config.canId               = doc["canId"]               | config.canId;
      config.voltageDividerRatio = doc["voltageDividerRatio"] | config.voltageDividerRatio;
      config.apName              = doc["apName"]              | config.apName;
      config.apPassword          = doc["apPassword"]          | config.apPassword;
      
      // Load AP network configuration
      if (doc.containsKey("apIP") && doc.containsKey("apGateway") && doc.containsKey("apSubnet") &&
          doc["apIP"].is<JsonArray>() && doc["apGateway"].is<JsonArray>() && doc["apSubnet"].is<JsonArray>()) {
        JsonArray apIP = doc["apIP"];
        JsonArray apGateway = doc["apGateway"];
        JsonArray apSubnet = doc["apSubnet"];
        for (int i = 0; i < 4 && i < apIP.size(); i++) {
          config.apIP[i] = apIP[i] | config.apIP[i];
          config.apGateway[i] = apGateway[i] | config.apGateway[i];
          config.apSubnet[i] = apSubnet[i] | config.apSubnet[i];
        }
      }
      addLog(F("Config loaded"));
    } else {
      addLog(F("! Failed to parse config – using defaults"));
    }
    f.close();
  }
}

// =============================================================================
//  Power‑supply ADC helper
// =============================================================================
float readPowerVoltage() {
  metrics.rawVoltage = analogRead(POWER_MONITOR_PIN) * 3.3f / 4095.0f;
  return metrics.rawVoltage * config.voltageDividerRatio;
}

// =============================================================================
//  CAN helpers
// =============================================================================
// Add with other global variables
static bool spiInitialized = false;

bool setupCANController() {
  Serial.println(F("Initializing MCP2515…"));

  hardwareReset();
  delay(100);
    
  // Initialize SPI only if not already initialized
  if (!spiInitialized) {
    SPI.begin(VSPI_SCLK, VSPI_MISO, VSPI_MOSI, CAN_CS);
    SPI.setFrequency(SPI_FREQUENCY);
    spiInitialized = true;
  }
  pinMode(CAN_CS, OUTPUT);
  digitalWrite(CAN_CS, HIGH);

  // Initialize CAN
  if (CAN.begin(CAN_RATE, MCP_CONFIG, MCP_16MHZ) != CAN_OK) {
    Serial.println(F("! MCP2515 init failed"));
    return false;
  }

  // Enter config mode
  if (CAN.setMode(MCP_CONFIG) != CAN_OK) {
    Serial.println(F("! Could not enter CONFIG mode"));
    return false;
  }

  // Clear any pending errors
  writeRegister(MCP_EFLG, 0);

  // Set up filters and masks
  for (uint8_t f = 0; f < 6; ++f) CAN.init_Filt(f, 0, 0x7FF);
  for (uint8_t m = 0; m < 2; ++m) CAN.init_Mask(m, 0, 0x7FF);

  // Clear TX buffers
  writeRegister(MCP_TXB0CTRL, 0);
  writeRegister(MCP_TXB1CTRL, 0);
  writeRegister(MCP_TXB2CTRL, 0);

  // Enter normal mode
  if (CAN.setMode(MCP_NORMAL) != CAN_OK) {
    Serial.println(F("! Could not enter NORMAL mode"));
    return false;
  }

  // Verify normal mode
  if ((readRegister(MCP_CANSTAT) & MODE_MASK) != MCP_NORMAL) {
    Serial.println(F("! Normal mode verification failed"));
    return false;
  }

  Serial.println(F("MCP2515 ready"));
            canInitialized = true;
  return true;
}

void sendCANMessage() {
  if (!canInitialized) return;

  uint8_t frame[8] = {0};
  
  // Pack voltage (mV)
  uint16_t mV = bmsData.voltage * 1000;
  frame[0] = mV >> 8; frame[1] = mV & 0xFF;
  
  // Pack current (0.1A resolution, allowing ±200A range)
  int16_t amps = (int16_t)roundf(constrain(bmsData.current, -200.0f, 200.0f) * 10.0f);
  frame[2] = amps >> 8; frame[3] = amps & 0xFF;
  
  // Pack SOC (0-100%)
  frame[4] = bmsData.soc;
  
  // Pack temperature with safety limits
  int8_t tempC = constrain(bmsData.temperature, -40, 215);
  frame[5] = tempC + 40;
  
  // Pack status flags
  frame[6] = (bmsData.chargeEnabled ? 0x01 : 0) |
             (bmsData.dischargeEnabled ? 0x02 : 0) |
             (bmsData.faultFlags ? 0x04 : 0);
    
  for (uint8_t retry = 0; retry < 3; ++retry) {
    if (sendCANMessageWithParams(config.canId, frame, 8) == CAN_OK) return;
    delay(10);
  }
  addLog(F("! Failed to send CAN frame"));
  canError = true;
}

byte sendCANMessageWithParams(unsigned long id, byte *data, byte len) {
  if (!canInitialized) return CAN_FAIL;
  if (len == 0 || len > 8) return CAN_FAIL;  // Defensive length check
  // Use extended frame format (1) for IDs > 0x7FF, standard format (0) otherwise
  byte ext = (id > 0x7FF) ? 1 : 0;
  byte res = CAN.sendMsgBuf(id, ext, len, data);
  if (res == CAN_OK) {
    canActive = true;
    lastCanSend = millis();
  } else {
    canError = true;
  }
  return res;
}

void monitorMessages() {
  if (CAN.checkReceive() != CAN_MSGAVAIL) return;
  byte rxLen = 0;
  byte buf[8];
  unsigned long id = 0;
  byte ext = 0;
  if (CAN.readMsgBuf(&id, &ext, &rxLen, buf) == CAN_OK) {
    Serial.printf("→ 0x%03lX (%d) ", id, rxLen);
    for (byte i = 0; i < rxLen; ++i) Serial.printf("%02X ", buf[i]);
    Serial.println();
  }
}

bool sendTestMessage() {
  if (!canInitialized) return false;
  
  // Send test message
  byte data[8] = {0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88};
  if (sendCANMessageWithParams(TEST_MSG_ID, data, 8) != CAN_OK) {
    addLog(F("Test message send failed"));
    return false;
  }

  // Wait for echo
  unsigned long start = millis();
  while (millis() - start < 100) {
    if (CAN.checkReceive() == CAN_MSGAVAIL) {
      byte rxLen = 0;
      byte buf[8];
      unsigned long id;
      byte ext;
      if (CAN.readMsgBuf(&id, &ext, &rxLen, buf) == CAN_OK) {
        if (id == TEST_MSG_ID && rxLen == 8) {
          // Verify echo data
          for (int i = 0; i < 8; i++) {
            if (buf[i] != data[i]) {
              addLog(F("Test message echo data mismatch"));
              return false;
            }
          }
          return true;
        }
      }
    }
    delay(1);
  }
  addLog(F("Test message echo timeout"));
  return false;
}

void checkBusStatus() {
  uint8_t eflg = readRegister(MCP_EFLG);
  uint8_t tec = readRegister(MCP_TEC);
  uint8_t rec = readRegister(MCP_REC);
  
  canError = (eflg != 0);
  if (canError) {
    addLog(String(F("CAN Error: EFLG=0x")) + String(eflg, HEX) + 
           F(", TEC=") + String(tec) + 
           F(", REC=") + String(rec));
  }
}

// =============================================================================
//  Wi‑Fi, web‑server & OTA helpers -------------------------------------------
// =============================================================================
void setupWiFi() {
  addLog(F("Starting WiFi"));
  WiFi.mode(WIFI_AP_STA);
  delay(1000);
  WiFi.setHostname("InverterBatteryBridge");

  // Configure AP immediately to prevent race condition
  IPAddress apIP(config.apIP[0], config.apIP[1], config.apIP[2], config.apIP[3]);
  IPAddress apGateway(config.apGateway[0], config.apGateway[1], config.apGateway[2], config.apGateway[3]);
  IPAddress apSubnet(config.apSubnet[0], config.apSubnet[1], config.apSubnet[2], config.apSubnet[3]);
  WiFi.softAPConfig(apIP, apGateway, apSubnet);

  wifiManager.setDebugOutput(true);
  wifiManager.setConfigPortalTimeout(0);
  wifiManager.setConnectTimeout(20);
  wifiManager.setAPStaticIPConfig(apIP, apGateway, apSubnet);

  if (config.apName.isEmpty()) {
    String suf = String(random(1000,9999));
    config.apName     = "InverterBatteryBridge-" + suf;
    config.apPassword = "setup" + suf;
    saveConfig();
  }

  if (!wifiManager.autoConnect(config.apName.c_str(), config.apPassword.c_str())) {
    addLog(F("AutoConnect failed – rebooting"));
    delay(3000);
    ESP.restart();
  }
  addLog("Connected – IP " + WiFi.localIP().toString());
}

void setupWebServer() {
  addLog(F("Starting web server"));
  server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest *req){ 
    req->send(200,"application/json", getSystemStatus()); 
  });
  
  server.on("/api/config", HTTP_GET, [](AsyncWebServerRequest *req){ 
    req->send(200,"application/json", getSystemConfig()); 
  });
  
  server.on("/api/config", HTTP_POST, [](AsyncWebServerRequest *req){}, NULL,
    [](AsyncWebServerRequest *req, uint8_t *data, size_t len, size_t index, size_t total) {
      StaticJsonDocument<384> doc;
      DeserializationError error = deserializeJson(doc, data, len);
      if (!error) {
        config.batteryCapacity = doc["batteryCapacity"] | config.batteryCapacity;
        config.moduleCount = doc["moduleCount"] | config.moduleCount;
        config.canInterval = doc["canInterval"] | config.canInterval;
        config.alarmEnabled = doc["alarmEnabled"] | config.alarmEnabled;
        config.canId = doc["canId"] | config.canId;
        saveConfig();
        req->send(200, "application/json", "{\"success\":true}");
      } else {
        req->send(400, "application/json", "{\"success\":false,\"error\":\"Invalid JSON\"}");
      }
    }
  );

  server.on("/api/logs", HTTP_GET, [](AsyncWebServerRequest *req){
    String logs = "";
    int idx = logIndex;
    for (int i = 0; i < LOG_BUFFER_SIZE; i++) {
      int j = (idx + i) % LOG_BUFFER_SIZE;
      if (logBuffer[j].length() > 0) {
        logs += logBuffer[j] + "\n";
      }
    }
    req->send(200, "text/plain", logs);
  });

  server.on("/metrics", HTTP_GET, [](AsyncWebServerRequest *req){
    metrics.txErrors = readRegister(MCP_TEC);
    metrics.rxErrors = readRegister(MCP_REC);
    metrics.wifiRSSI = WiFi.RSSI();
    metrics.supplyVoltage = readPowerVoltage();
    metrics.lastUpdate = millis();

    String res;
    // Gauge metrics for error counters (since they can decrease)
    res += "# HELP can_tx_errors Current number of CAN transmit errors\n";
    res += "# TYPE can_tx_errors gauge\n";
    res += "can_tx_errors " + String(metrics.txErrors) + "\n\n";
    
    res += "# HELP can_rx_errors Current number of CAN receive errors\n";
    res += "# TYPE can_rx_errors gauge\n";
    res += "can_rx_errors " + String(metrics.rxErrors) + "\n\n";
    
    // Error state metrics
    res += "# HELP can_error_state Current CAN error state (0=OK, 1=Error)\n";
    res += "# TYPE can_error_state gauge\n";
    res += "can_error_state " + String(canError ? 1 : 0) + "\n\n";
    
    res += "# HELP wifi_connected Current WiFi connection state (0=Disconnected, 1=Connected)\n";
    res += "# TYPE wifi_connected gauge\n";
    res += "wifi_connected " + String(WiFi.status() == WL_CONNECTED ? 1 : 0) + "\n\n";
    
    // Other gauge metrics
    res += "# HELP wifi_rssi Current WiFi signal strength in dBm\n";
    res += "# TYPE wifi_rssi gauge\n";
    res += "wifi_rssi " + String(metrics.wifiRSSI) + "\n\n";
    
    res += "# HELP supply_voltage Current power supply voltage in volts\n";
    res += "# TYPE supply_voltage gauge\n";
    res += "supply_voltage " + String(metrics.supplyVoltage,2) + "\n\n";
    
    res += "# HELP last_update Timestamp of last metrics update in milliseconds\n";
    res += "# TYPE last_update gauge\n";
    res += "last_update " + String(metrics.lastUpdate) + "\n";
    req->send(200, "text/plain", res);
  });

  server.on("/api/voltage", HTTP_GET, [](AsyncWebServerRequest *req){
    readPowerVoltage();  // Update the cache
    StaticJsonDocument<128> doc;
    doc["raw"] = metrics.rawVoltage;
    doc["actual"] = metrics.rawVoltage * config.voltageDividerRatio;
    doc["dividerRatio"] = config.voltageDividerRatio;
    String out; 
    serializeJson(doc,out);
    req->send(200,"application/json", out);
  });

  server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");
  server.onNotFound([](AsyncWebServerRequest *req){ 
    req->send(404,"text/plain","Not Found"); 
  });
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin","*");
  server.begin();
}

// =============================================================================
//  Arduino setup() & loop() ---------------------------------------------------
// =============================================================================
void setup() {
  Serial.begin(115200);
  Serial.println(F("\n=== Inverter Battery Bridge ==="));

  SPI.begin(VSPI_SCLK, VSPI_MISO, VSPI_MOSI, CAN_CS);
  SPI.setFrequency(SPI_FREQUENCY);
  spiInitialized = true;  // Mark SPI as initialized

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(CAN_CS, OUTPUT);
  pinMode(CAN_INT, INPUT);
  pinMode(RS485_DE, OUTPUT);
  digitalWrite(RS485_DE, LOW);

  if (!LittleFS.begin(true)) Serial.println(F("! LittleFS mount failed"));
  loadConfig();
  setupWiFi();

  esp_task_wdt_init(WDT_TIMEOUT,true);
  esp_task_wdt_add(nullptr);

  setupWebServer();
  setupCAN();

  Serial.println(F("Setup complete"));
}

void loop() {
  esp_task_wdt_reset();

  static unsigned long lastStatus   = 0;
  static unsigned long lastTestMsg  = 0;
  static unsigned long lastPoll     = 0;
  unsigned long now = millis();

  monitorMessages();

  if ((!canInitialized || canError) && (now - recoveryState.lastCanRecovery) >= RECOVERY_DELAY) {
    attemptCANRecovery();
  }
  if (WiFi.status() != WL_CONNECTED && (now - recoveryState.lastWifiRecovery) >= RECOVERY_DELAY) {
    attemptWiFiRecovery();
  }

  if (now - lastPoll >= config.canInterval) {
    if (!canInitialized) attemptCANRecovery();
    sendCANMessage();
    lastPoll = now;
  }
  if (now - lastStatus >= 5000) {
    checkBusStatus();
    lastStatus = now;
  }
  if (now - lastTestMsg >= 1000) {
    sendTestMessage();
    lastTestMsg = now;
  }

  if (now - lastLedBlink >= LED_BLINK_INTERVAL) {
    digitalWrite(LED_BUILTIN, canInitialized ? !digitalRead(LED_BUILTIN) : HIGH);
    lastLedBlink = now;
  }
}

// Add near the top with other defines
#define CAN_ERR_OK      0
#define CAN_ERR_FAIL    1
#define CAN_ERR_TX      2
#define CAN_ERR_RX      3
#define CAN_ERR_BUSOFF  4
#define CAN_ERR_PASSIVE 5

bool validateCANMessage(unsigned long id, byte len, bool ext = false) {
    if (!ext && id > 0x7FF) return false;      // Standard frame: 11 bits
    if (ext && id > 0x1FFFFFFF) return false;  // Extended frame: 29 bits
    if (len > 8) return false;                 // Max 8 bytes
    return true;
}

void resetRecoveryState() {
    recoveryState.canRecoveryAttempts = 0;
    recoveryState.wifiRecoveryAttempts = 0;
    recoveryState.lastCanRecovery = 0;
    recoveryState.lastWifiRecovery = 0;
    recoveryState.inRecovery = false;
}

void setupCAN() {               // legacy wrapper
    setupCANController();       // (keeps earlier calls working)
}
