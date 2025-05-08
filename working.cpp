#include <Arduino.h>
#include <SPI.h>
#include <mcp_can.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>

// On most ESP32 dev boards, GPIO2 is connected to an internal LED
// Some boards use GPIO5, others use GPIO22
#define LED_BUILTIN 22  // Try a different LED pin
#define CAN_CS 5
#define VSPI_MISO 19
#define VSPI_MOSI 23
#define VSPI_SCLK 18
#define POWER_MONITOR_PIN 32  // ADC1_CH4 (GPIO32)

// MCP2515 Register addresses
#define MCP_CANSTAT     0x0E
#define MCP_CANCTRL     0x0F
#define MCP_CNF1        0x2A
#define MCP_CNF2        0x29
#define MCP_CNF3        0x28
#define MCP_TXB0CTRL    0x30
#define MCP_TXB1CTRL    0x40
#define MCP_TXB2CTRL    0x50
#define MCP_RXB0CTRL    0x60
#define MCP_RXB1CTRL    0x70
#define MCP_EFLG        0x2D  // Error Flag Register
#define MCP_TEC         0x1C  // Transmit Error Counter
#define MCP_REC         0x1D  // Receive Error Counter

// MCP2515 Commands
#define MCP_RESET       0xC0
#define MCP_READ        0x03
#define MCP_WRITE       0x02
#define MCP_BITMOD      0x05
#define MCP_LOAD_TX0    0x40
#define MCP_LOAD_TX1    0x42
#define MCP_LOAD_TX2    0x44
#define MCP_RTS_TX0     0x81
#define MCP_RTS_TX1     0x82
#define MCP_RTS_TX2     0x84
#define MCP_READ_RX0    0x90
#define MCP_READ_RX1    0x94
#define MCP_READ_STATUS 0xA0
#define MCP_RX_STATUS   0xB0

// MCP2515 Modes
#define MCP_NORMAL      0x00
#define MCP_SLEEP       0x20
#define MCP_LOOPBACK    0x40
#define MCP_LISTENONLY  0x60
#define MCP_CONFIG      0x80

// Configuration values for 500kbps at 16MHz
#define CNF1_500KBPS    0x00  // SJW=1, BRP=0
#define CNF2_500KBPS    0x90  // BTLMODE=1, SAM=0, PHSEG1=2, PRSEG=2
#define CNF3_500KBPS    0x02  // PHSEG2=2

// Test message ID and data
#define TEST_MSG_ID     0x123
#define TEST_MSG_LEN    8
#define TEST_MSG_DATA   {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88}

MCP_CAN CAN(CAN_CS);
AsyncWebServer server(80);
WiFiManager wifiManager;

// Configuration structure
struct Config {
    float batteryCapacity;  // kWh
    uint8_t moduleCount;    // Number of modules
    uint16_t canInterval;   // ms
    bool alarmEnabled;
    uint32_t canId;
} config = {
    .batteryCapacity = 17.4,
    .moduleCount = 7,
    .canInterval = 1000,
    .alarmEnabled = true,
    .canId = 0x4210
};

#define CONFIG_FILE "/config.json"

// --- Logging Buffer ---
#define LOG_BUFFER_SIZE 1000
String logBuffer[LOG_BUFFER_SIZE];
int logIndex = 0;

void addLog(const String &msg) {
    logBuffer[logIndex] = String(millis()) + ": " + msg;
    logIndex = (logIndex + 1) % LOG_BUFFER_SIZE;
    Serial.println(msg);
}

// Function to read power supply voltage
float readPowerVoltage() {
    int rawValue = analogRead(POWER_MONITOR_PIN);
    // Convert ADC reading to voltage (assuming 3.3V reference)
    // Multiply by 10.57 to account for voltage divider ratio and calibration
    float voltage = (rawValue * 3.3 * 10) / 4095.0;
    return voltage;
}

// Function to test power supply
void testPowerSupply() {
    Serial.println("\nDetailed Power Supply Test:");
    float voltage = readPowerVoltage();
    Serial.printf("Raw ADC reading: %d\n", analogRead(POWER_MONITOR_PIN));
    Serial.printf("Calculated voltage: %.2fV\n", voltage);
    Serial.println("Note: If this reading is incorrect, please verify with a multimeter");
}

// Function to read MCP2515 register
byte readRegister(byte address) {
    digitalWrite(CAN_CS, LOW);
    SPI.transfer(MCP_READ);
    SPI.transfer(address);
    byte value = SPI.transfer(0x00);
    digitalWrite(CAN_CS, HIGH);
    return value;
}

// Function to write MCP2515 register
void writeRegister(byte address, byte value) {
    digitalWrite(CAN_CS, LOW);
    SPI.transfer(MCP_WRITE);
    SPI.transfer(address);
    SPI.transfer(value);
    digitalWrite(CAN_CS, HIGH);
}

// Function to set MCP2515 mode
byte setMode(byte mode) {
    writeRegister(MCP_CANCTRL, mode);
    delay(10);  // Wait for mode change
    return readRegister(MCP_CANSTAT) & 0xE0;  // Return current mode
}

// Function to perform software reset
void softwareReset() {
    Serial.println("Performing software reset...");
    digitalWrite(CAN_CS, LOW);
    SPI.transfer(MCP_RESET);
    digitalWrite(CAN_CS, HIGH);
    delay(100);  // Wait for reset to complete
}

// Function to check CAN bus status
void checkBusStatus() {
    byte eflg = readRegister(MCP_EFLG);
    byte tec = readRegister(MCP_TEC);
    byte rec = readRegister(MCP_REC);
    
    Serial.println("\nCAN Bus Status:");
    Serial.printf("Error Flags: 0x%02X\n", eflg);
    Serial.printf("Transmit Error Counter: %d\n", tec);
    Serial.printf("Receive Error Counter: %d\n", rec);
    
    // Check for specific error conditions
    if (eflg & 0x80) Serial.println("Error: Bus-off state");
    if (eflg & 0x40) Serial.println("Error: TX Error Warning");
    if (eflg & 0x20) Serial.println("Error: RX Error Warning");
    if (eflg & 0x10) Serial.println("Error: TX Error Passive");
    if (eflg & 0x08) Serial.println("Error: RX Error Passive");
    if (eflg & 0x04) Serial.println("Error: TX Bus Error");
    if (eflg & 0x02) Serial.println("Error: RX Bus Error");
    if (eflg & 0x01) Serial.println("Error: Message Error");
}

// Function to send test message
bool sendTestMessage() {
    byte testData[TEST_MSG_LEN] = TEST_MSG_DATA;
    
    Serial.println("\nSending Test Message:");
    Serial.printf("ID: 0x%03X, Length: %d\n", TEST_MSG_ID, TEST_MSG_LEN);
    Serial.print("Data: ");
    for(int i = 0; i < TEST_MSG_LEN; i++) {
        Serial.printf("0x%02X ", testData[i]);
    }
    Serial.println();
    
    // Check CAN controller status before sending
    byte status = readRegister(MCP_CANSTAT);
    Serial.printf("CAN Status before send: 0x%02X\n", status);
    
    // Check if we're in normal mode
    if ((status & 0xE0) != MCP_NORMAL) {
        Serial.println("Error: CAN controller not in normal mode");
        return false;
    }
    
    // Check transmit buffer status
    byte txStatus = readRegister(MCP_TXB0CTRL);
    Serial.printf("TX Buffer Status: 0x%02X\n", txStatus);
    
    // Try to send the message
    Serial.println("Attempting to send message...");
    byte result = CAN.sendMsgBuf(TEST_MSG_ID, 0, TEST_MSG_LEN, testData);
    
    // Check CAN controller status after sending
    status = readRegister(MCP_CANSTAT);
    Serial.printf("CAN Status after send: 0x%02X\n", status);
    
    // Check transmit buffer status after sending
    txStatus = readRegister(MCP_TXB0CTRL);
    Serial.printf("TX Buffer Status after send: 0x%02X\n", txStatus);
    
    if (result == CAN_OK) {
        Serial.println("Message sent successfully");
        return true;
    } else {
        Serial.printf("Failed to send message, error code: %d\n", result);
        return false;
    }
}

// Function to monitor received messages
void monitorMessages() {
    if (CAN.checkReceive() == CAN_MSGAVAIL) {
        byte len = 0;
        byte buf[8];
        unsigned long id = 0;
        byte ext = 0;
        
        if (CAN.readMsgBuf(&id, &ext, &len, buf) == CAN_OK) {
            Serial.println("\nReceived Message:");
            Serial.printf("ID: 0x%03X, Extended: %d, Length: %d\n", id, ext, len);
            Serial.print("Data: ");
            for(int i = 0; i < len; i++) {
                Serial.printf("0x%02X ", buf[i]);
            }
            Serial.println();
        }
    }
}

// Function to initialize CAN controller
bool initializeCAN() {
    Serial.println("\nInitializing CAN controller...");
    
    // 1. Software reset
    softwareReset();
    delay(100);
    
    // 2. Set configuration mode
    byte mode = setMode(MCP_CONFIG);
    Serial.printf("Mode after reset: 0x%02X\n", mode);
    if (mode != MCP_CONFIG) {
        Serial.println("Failed to enter configuration mode");
        return false;
    }
    
    // 3. Set configuration registers
    Serial.println("Setting configuration registers...");
    writeRegister(MCP_CNF1, CNF1_500KBPS);
    writeRegister(MCP_CNF2, CNF2_500KBPS);
    writeRegister(MCP_CNF3, CNF3_500KBPS);
    
    // 4. Verify configuration
    byte cnf1 = readRegister(MCP_CNF1);
    byte cnf2 = readRegister(MCP_CNF2);
    byte cnf3 = readRegister(MCP_CNF3);
    
    Serial.printf("CNF1: 0x%02X (Expected: 0x%02X)\n", cnf1, CNF1_500KBPS);
    Serial.printf("CNF2: 0x%02X (Expected: 0x%02X)\n", cnf2, CNF2_500KBPS);
    Serial.printf("CNF3: 0x%02X (Expected: 0x%02X)\n", cnf3, CNF3_500KBPS);
    
    if (cnf1 != CNF1_500KBPS || cnf2 != CNF2_500KBPS || cnf3 != CNF3_500KBPS) {
        Serial.println("Configuration register verification failed");
        return false;
    }
    
    // 5. Set normal mode
    mode = setMode(MCP_NORMAL);
    Serial.printf("Mode after configuration: 0x%02X\n", mode);
    if (mode != MCP_NORMAL) {
        Serial.println("Failed to enter normal mode");
        return false;
    }
    
    return true;
}

void saveConfig() {
    StaticJsonDocument<512> doc;
    doc["batteryCapacity"] = config.batteryCapacity;
    doc["moduleCount"] = config.moduleCount;
    doc["canInterval"] = config.canInterval;
    doc["alarmEnabled"] = config.alarmEnabled;
    doc["canId"] = config.canId;
    
    File file = LittleFS.open(CONFIG_FILE, "w");
    if (file) {
        serializeJson(doc, file);
        file.close();
        addLog("Config saved to LittleFS.");
    } else {
        addLog("Failed to open config file for writing!");
    }
}

void loadConfig() {
    if (LittleFS.exists(CONFIG_FILE)) {
        File file = LittleFS.open(CONFIG_FILE, "r");
        if (file) {
            StaticJsonDocument<512> doc;
            DeserializationError error = deserializeJson(doc, file);
            if (!error) {
                config.batteryCapacity = doc["batteryCapacity"] | 17.4;
                config.moduleCount = doc["moduleCount"] | 7;
                config.canInterval = doc["canInterval"] | 1000;
                config.alarmEnabled = doc["alarmEnabled"] | true;
                config.canId = doc["canId"] | 0x4210;
                addLog("Config loaded from LittleFS.");
            } else {
                addLog("Failed to parse config file!");
            }
            file.close();
        } else {
            addLog("Failed to open config file for reading!");
        }
    } else {
        addLog("Config file does not exist, using defaults.");
    }
}

void testConfig() {
    addLog("=== Testing Configuration System ===");
    
    // Print current configuration
    addLog("Current configuration:");
    addLog("Battery Capacity: " + String(config.batteryCapacity, 1) + " kWh");
    addLog("Module Count: " + String(config.moduleCount));
    addLog("CAN Interval: " + String(config.canInterval) + " ms");
    addLog("Alarm Enabled: " + String(config.alarmEnabled ? "Yes" : "No"));
    addLog("CAN ID: 0x" + String(config.canId, HEX));
    
    // Modify some values
    addLog("Modifying configuration...");
    config.batteryCapacity = 20.0;
    config.moduleCount = 8;
    config.canInterval = 2000;
    
    // Save the modified configuration
    addLog("Saving modified configuration...");
    saveConfig();
    
    // Reset to defaults
    addLog("Resetting to defaults...");
    config = {
        .batteryCapacity = 17.4,
        .moduleCount = 7,
        .canInterval = 1000,
        .alarmEnabled = true,
        .canId = 0x4210
    };
    
    // Load the saved configuration
    addLog("Loading saved configuration...");
    loadConfig();
    
    // Print final configuration
    addLog("Final configuration after save/load cycle:");
    addLog("Battery Capacity: " + String(config.batteryCapacity, 1) + " kWh");
    addLog("Module Count: " + String(config.moduleCount));
    addLog("CAN Interval: " + String(config.canInterval) + " ms");
    addLog("Alarm Enabled: " + String(config.alarmEnabled ? "Yes" : "No"));
    addLog("CAN ID: 0x" + String(config.canId, HEX));
    addLog("=== Configuration Test Complete ===");
}

void testLogging() {
    addLog("=== Testing Logging System ===");
    
    // Test basic logging
    addLog("Testing basic log message");
    
    // Test logging with different data types
    addLog("Testing integer: " + String(42));
    addLog("Testing float: " + String(3.14159, 5));
    addLog("Testing boolean: " + String(true));
    
    // Test logging with millis() timestamp
    unsigned long startTime = millis();
    addLog("Testing timestamp - this message should show current millis()");
    delay(1000); // Wait 1 second
    addLog("One second later - millis() should be ~1000 higher");
    
    // Test buffer wrapping
    addLog("Testing buffer wrapping...");
    for(int i = 0; i < LOG_BUFFER_SIZE + 10; i++) {
        logBuffer[logIndex] = "Buffer entry " + String(i);
        logIndex = (logIndex + 1) % LOG_BUFFER_SIZE;
    }
    addLog("Buffer filling complete");
    
    // Print some statistics
    addLog("Log buffer size: " + String(LOG_BUFFER_SIZE));
    addLog("Current log index: " + String(logIndex));
    
    addLog("=== Logging System Test Complete ===");
}

// BMS Data structure
struct BMSData {
    float voltage;
    float current;
    uint8_t soc;
    int8_t temperature;
    bool chargeEnabled;
    bool dischargeEnabled;
    uint16_t faultFlags;
} bmsData;

// Helper function to get system status as JSON
String getSystemStatus() {
    StaticJsonDocument<256> doc;
    doc["voltage"] = bmsData.voltage;
    doc["current"] = bmsData.current;
    doc["soc"] = bmsData.soc;
    doc["temperature"] = bmsData.temperature;
    doc["chargeEnabled"] = bmsData.chargeEnabled;
    doc["dischargeEnabled"] = bmsData.dischargeEnabled;
    doc["wifiConnected"] = (WiFi.status() == WL_CONNECTED);
    String response;
    serializeJson(doc, response);
    return response;
}

// Helper function to get system config as JSON
String getSystemConfig() {
    StaticJsonDocument<256> doc;
    doc["batteryCapacity"] = config.batteryCapacity;
    doc["moduleCount"] = config.moduleCount;
    doc["canInterval"] = config.canInterval;
    doc["alarmEnabled"] = config.alarmEnabled;
    doc["canId"] = config.canId;
    String response;
    serializeJson(doc, response);
    return response;
}

void setupWebServer() {
    Serial.println("Starting web server...");
    
    // API endpoints
    server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest *request) {
        StaticJsonDocument<256> doc;
        doc["voltage"] = bmsData.voltage;
        doc["current"] = bmsData.current;
        doc["soc"] = bmsData.soc;
        doc["temperature"] = bmsData.temperature;
        doc["chargeEnabled"] = bmsData.chargeEnabled;
        doc["dischargeEnabled"] = bmsData.dischargeEnabled;
        doc["wifiConnected"] = (WiFi.status() == WL_CONNECTED);
        String response;
        serializeJson(doc, response);
        request->send(200, "application/json", response);
    });

    server.on("/api/logs", HTTP_GET, [](AsyncWebServerRequest *request) {
        String logs = "";
        int idx = logIndex;
        for (int i = 0; i < LOG_BUFFER_SIZE; i++) {
            int j = (idx + i) % LOG_BUFFER_SIZE;
            if (logBuffer[j].length() > 0) {
                logs += logBuffer[j] + "\n";
            }
        }
        request->send(200, "text/plain", logs);
    });

    server.on("/api/config", HTTP_GET, [](AsyncWebServerRequest *request) {
        StaticJsonDocument<256> doc;
        doc["batteryCapacity"] = config.batteryCapacity;
        doc["moduleCount"] = config.moduleCount;
        doc["canInterval"] = config.canInterval;
        doc["alarmEnabled"] = config.alarmEnabled;
        doc["canId"] = config.canId;
        String response;
        serializeJson(doc, response);
        request->send(200, "application/json", response);
    });

    server.on("/api/config", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL,
        [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
            StaticJsonDocument<256> doc;
            DeserializationError error = deserializeJson(doc, data, len);
            if (!error) {
                config.batteryCapacity = doc["batteryCapacity"] | config.batteryCapacity;
                config.moduleCount = doc["moduleCount"] | config.moduleCount;
                config.canInterval = doc["canInterval"] | config.canInterval;
                config.alarmEnabled = doc["alarmEnabled"] | config.alarmEnabled;
                config.canId = doc["canId"] | config.canId;
                saveConfig();
                request->send(200, "application/json", "{\"success\":true}");
            } else {
                request->send(400, "application/json", "{\"success\":false,\"error\":\"Invalid JSON\"}");
            }
        }
    );

    // Serve static files from LittleFS
    server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");

    // Handle 404 errors
    server.onNotFound([](AsyncWebServerRequest *request) {
        if (request->method() == HTTP_OPTIONS) {
            request->send(200);
        } else {
            request->send(404, "text/plain", "Not found");
        }
    });

    // Enable CORS
    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Methods", "GET, POST, PUT, DELETE, OPTIONS");
    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Headers", "Content-Type");

    server.begin();
    Serial.println("Web server ready");
    Serial.printf("Web interface available at: http://%s\n", WiFi.localIP().toString().c_str());
}

void setupWiFi() {
    addLog("=== Starting WiFi Setup ===");
    
    // Set WiFi mode to both AP and Station
    addLog("Setting WiFi mode to AP+STA...");
    WiFi.mode(WIFI_AP_STA);
    delay(1000);  // Give WiFi time to initialize

    // Set device hostname
    addLog("Setting hostname to: InverterBatteryBridge");
    WiFi.setHostname("InverterBatteryBridge");

    // Configure WiFiManager
    addLog("Configuring WiFiManager...");
    wifiManager.setDebugOutput(true);  // Enable debug output
    wifiManager.setConfigPortalTimeout(0);  // Disable timeout
    wifiManager.setConnectTimeout(20);
    wifiManager.setAPStaticIPConfig(IPAddress(192,168,4,1), IPAddress(192,168,4,1), IPAddress(255,255,255,0));
    
    // Set AP name and password
    String apName = "InverterBatteryBridge-Setup";
    String apPassword = "setup1234";
    
    addLog("Creating access point: " + apName + " (Password: " + apPassword + ")");
    addLog("AP IP will be: 192.168.4.1");
    
    if (!wifiManager.autoConnect(apName.c_str(), apPassword.c_str())) {
        addLog("Failed to connect and hit timeout");
        delay(3000);
        ESP.restart();
    }

    addLog("WiFi connected successfully");
    addLog("IP Address: " + WiFi.localIP().toString());
    addLog("=== WiFi Setup Complete ===");
}

void setup() {
    // Initialize serial with basic configuration
    Serial.begin(115200);
    delay(2000);  // Give serial more time to stabilize
    Serial.println("\n\n=== InverterBatteryBridge Starting ===");
    Serial.println("Version: 1.0.0");
    
    // Initialize LittleFS
    if (!LittleFS.begin(true)) {
        Serial.println("LittleFS Mount Failed");
        return;
    }

    // Load configuration
    addLog("Loading configuration...");
    loadConfig();
    
    // Give system time to stabilize before WiFi setup
    delay(2000);
    addLog("System stabilized, starting WiFi setup...");
    
    // Setup WiFi first
    setupWiFi();
    setupWebServer();
    
    // Initialize power monitoring
    pinMode(POWER_MONITOR_PIN, INPUT);
    
    // Initialize LED
    addLog("Setting up LED...");
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    
    // Test LED
    addLog("Testing LED with 3 blinks...");
    for(int i = 0; i < 3; i++) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(200);
        digitalWrite(LED_BUILTIN, LOW);
        delay(200);
    }
    
    // Run detailed power supply test
    testPowerSupply();
    
    // Initialize SPI
    addLog("\nInitializing SPI...");
    pinMode(CAN_CS, OUTPUT);
    digitalWrite(CAN_CS, HIGH);  // Deselect CAN controller
    
    // Initialize other SPI pins explicitly with pull-ups
    pinMode(VSPI_MISO, INPUT_PULLUP);
    pinMode(VSPI_MOSI, OUTPUT);
    pinMode(VSPI_SCLK, OUTPUT);
    
    // Initialize SPI with slower speed and different mode
    SPI.begin(VSPI_SCLK, VSPI_MISO, VSPI_MOSI, CAN_CS);
    SPI.beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE0));  // Reduced to 100kHz
    
    addLog("SPI initialized");
    
    // Initialize CAN with detailed debugging
    addLog("\nStarting CAN initialization...");
    addLog("Pins in use:");
    addLog("CS: " + String(CAN_CS) + ", MISO: " + String(VSPI_MISO) + ", MOSI: " + String(VSPI_MOSI) + ", SCK: " + String(VSPI_SCLK));
    
    if (initializeCAN()) {
        addLog("CAN initialized successfully!");
        digitalWrite(LED_BUILTIN, HIGH);  // Keep LED on if successful
        
        // Send test message
        if (sendTestMessage()) {
            addLog("Test message sent successfully");
        } else {
            addLog("Failed to send test message");
        }
    } else {
        addLog("Failed to initialize CAN");
        addLog("Possible causes:");
        addLog("1. Incorrect wiring");
        addLog("2. CAN controller not responding");
        addLog("3. SPI communication issues");
        addLog("4. Power supply problems");
        // Blink rapidly 10 times to indicate initialization failure
        for(int i = 0; i < 10; i++) {
            digitalWrite(LED_BUILTIN, HIGH);
            delay(50);
            digitalWrite(LED_BUILTIN, LOW);
            delay(50);
        }
    }
}

void loop() {
    static unsigned long lastStatusCheck = 0;
    static unsigned long lastTestMessage = 0;
    const unsigned long STATUS_INTERVAL = 5000;  // Check status every 5 seconds
    const unsigned long TEST_INTERVAL = 1000;    // Send test message every second
    
    // Monitor for received messages
    monitorMessages();
    
    // Check bus status periodically
    if (millis() - lastStatusCheck >= STATUS_INTERVAL) {
        checkBusStatus();
        lastStatusCheck = millis();
    }
    
    // Send test message periodically
    if (millis() - lastTestMessage >= TEST_INTERVAL) {
        if (sendTestMessage()) {
            digitalWrite(LED_BUILTIN, HIGH);
            delay(50);
            digitalWrite(LED_BUILTIN, LOW);
        }
        lastTestMessage = millis();
    }
    
    // Only blink if CAN initialization failed
    if (digitalRead(LED_BUILTIN) == LOW) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(1000);
        digitalWrite(LED_BUILTIN, LOW);
        delay(1000);
        Serial.println("Blink! (CAN init failed)");
    }
} 