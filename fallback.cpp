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

// On most ESP32 dev boards, GPIO2 is connected to an internal LED
// Some boards use GPIO5, others use GPIO22
#define LED_BUILTIN 22  // Try a different LED pin
#define CAN_CS 5
#define VSPI_MISO 19
#define VSPI_MOSI 23
#define VSPI_SCLK 18
#define POWER_MONITOR_PIN 32  // ADC1_CH4 (GPIO32)
#define CAN_INT 4  // Or whatever GPIO you use for MCP2515 INT

// Pin Definitions
// #define RS485_RX 16
// #define RS485_TX 17
// #define RS485_DE 25

// Constants
// #define JBD_BAUD 9600
#define CAN_SPEED MCP_16MHZ
#define CAN_RATE CAN_500KBPS
#define CAN_MODE MCP_NORMAL
#define SPI_CLOCK 8000000  // 8MHz SPI clock
// #define JBD_POLL_INTERVAL 1000  // 1 second
#define CONFIG_FILE "/config.json"

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

// Global Variables
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

// --- Logging Buffer ---
#define LOG_BUFFER_SIZE 1000
String logBuffer[LOG_BUFFER_SIZE];
int logIndex = 0;

void addLog(const String &msg) {
    logBuffer[logIndex] = String(millis()) + ": " + msg;
    logIndex = (logIndex + 1) % LOG_BUFFER_SIZE;
    Serial.println(msg);
}

// --- BMS Status Tracking ---
enum BMSStatus { BMS_OK, BMS_TIMEOUT, BMS_ERROR };
volatile BMSStatus lastBMSStatus = BMS_TIMEOUT;

// --- CAN Status Tracking ---
volatile bool canActive = false;
volatile bool canError = false;
unsigned long lastCanSend = 0;
volatile bool canInitialized = false;

// Function prototypes
void setupWiFi();
void setupWebServer();
void setupCAN();
// void setupRS485();
void loadConfig();
void saveConfig();
// void pollBMS();
void sendCANMessage();
void handleWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);

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

void setup() {
    // Initialize serial with proper configuration
    Serial.begin(115200);
    Serial.setRxBufferSize(1024);
    Serial.setTxBufferSize(1024);
    delay(1000);  // Wait for serial to stabilize
    Serial.println("\n\nJBD-Sungrow Bridge Starting...");
    Serial.println("Version: 1.0.0");

    // Initialize LittleFS
    if (!LittleFS.begin(true)) {
        Serial.println("LittleFS Mount Failed");
        return;
    }

    // Load configuration
    Serial.println("Calling loadConfig()...");
    loadConfig();

    // Setup hardware
    // setupRS485();
    setupCAN();
    
    // Setup WiFi and Web Server
    setupWiFi();
    setupWebServer();
}

void loop() {
    static unsigned long lastPoll = 0;
    static unsigned long lastCanCheck = 0;
    static int errorCount = 0;
    unsigned long currentMillis = millis();

    // Poll CAN at interval
    if (currentMillis - lastPoll >= config.canInterval) {
        if (!canInitialized) {
            Serial.println("CAN not initialized, attempting recovery...");
            setupCAN();
        }
        sendCANMessage();
        lastPoll = currentMillis;
    }

    // Handle CAN interrupts
    if (digitalRead(CAN_INT) == LOW) {
        unsigned long canId;
        byte len = 0;
        byte buf[8];
        
        // Read message from buffer
        byte error = CAN.readMsgBuf(&canId, &len, buf);
        if (error == CAN_OK) {
            // Successfully received a message
            canActive = true;
            errorCount = 0; // Reset error count on successful operation
            
            // Print received message for debugging
            Serial.printf("Received CAN message - ID: 0x%03X, Length: %d\n", canId, len);
            Serial.print("Data: ");
            for(int i = 0; i < len; i++) {
                Serial.printf("0x%02X ", buf[i]);
            }
            Serial.println();
        }
    }

    // Set canActive to false if no CAN message sent for 5 seconds
    if (millis() - lastCanSend > 5000) {
        canActive = false;
    }

    // Periodic CAN health check
    if (currentMillis - lastCanCheck > 5000) { // check every 5 seconds
        lastCanCheck = currentMillis;
        
        if (!canInitialized) {
            Serial.println("CAN not initialized (periodic check)");
            canError = true;
            setupCAN(); // Attempt recovery
        } else {
            // Check bus status
            checkBusStatus();
            
            // Try to send a test message to verify CAN is working
            uint8_t testData[8] = {0};
            byte sendResult = CAN.sendMsgBuf(0x7FF, 0, 8, testData);
            if (sendResult == CAN_OK) {
                errorCount = 0;
                canError = false;
                Serial.println("CAN health check passed");
            } else {
                errorCount++;
                Serial.printf("Failed to send test message. Result: %d\n", sendResult);
            }

            // Only attempt recovery if we've seen multiple consecutive errors
            if (errorCount >= 3) {
                Serial.println("Multiple CAN errors detected, attempting recovery...");
                canActive = false;
                canError = true;
                setupCAN(); // Attempt recovery
                errorCount = 0; // Reset error count after recovery attempt
            }
        }
    }
}

// void setupRS485() {
//     Serial2.begin(JBD_BAUD, SERIAL_8N1, RS485_RX, RS485_TX);
//     pinMode(RS485_DE, OUTPUT);
//     digitalWrite(RS485_DE, LOW);  // Start in receive mode
//     delay(10);  // Give the transceiver time to stabilize
// }

// void pollBMS() {
//     // RS485/BMS polling logic disabled for CAN-only testing
// }

void setupWiFi() {
    Serial.println("Starting WiFi setup...");
    WiFi.mode(WIFI_AP_STA);
    delay(1000);

    // Set device hostname
    WiFi.setHostname("InverterBatteryBridge");

    Serial.println("Starting WiFiManager...");
    wifiManager.setDebugOutput(true);  // Enable debug output
    wifiManager.setConfigPortalTimeout(0);  // Disable timeout
    wifiManager.setConnectTimeout(20);

    Serial.println("Creating access point...");
    if (!wifiManager.autoConnect("InverterBatteryBridge-Setup")) {
        Serial.println("Failed to connect and hit timeout");
        delay(3000);
        ESP.restart();
    }

    Serial.println("Connected to WiFi");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
}

void setupCAN() {
    Serial.println("\nInitializing CAN controller...");
    
    // Initialize SPI
    SPI.begin(VSPI_SCLK, VSPI_MISO, VSPI_MOSI, CAN_CS);
    SPI.setFrequency(SPI_CLOCK);
    
    // Set CS pin as output
    pinMode(CAN_CS, OUTPUT);
    digitalWrite(CAN_CS, HIGH);
    
    // 1. Software reset
    softwareReset();
    delay(100);
    
    // 2. Set configuration mode
    byte mode = setMode(MCP_CONFIG);
    if (mode != MCP_CONFIG) {
        Serial.println("Failed to enter configuration mode");
        canError = true;
        return;
    }
    Serial.println("Entered configuration mode");
    
    // 3. Set CAN timing registers
    writeRegister(MCP_CNF1, CNF1_500KBPS);
    writeRegister(MCP_CNF2, CNF2_500KBPS);
    writeRegister(MCP_CNF3, CNF3_500KBPS);
    
    // 4. Set filters and masks
    // Accept all messages
    writeRegister(MCP_RXB0CTRL, 0x60);  // Accept all messages
    writeRegister(MCP_RXB1CTRL, 0x60);  // Accept all messages
    
    // 5. Set normal mode
    mode = setMode(MCP_NORMAL);
    if (mode != MCP_NORMAL) {
        Serial.println("Failed to enter normal mode");
        canError = true;
        return;
    }
    Serial.println("Entered normal mode");
    
    // 6. Check bus status
    checkBusStatus();
    
    // 7. Send test message
    byte testData[TEST_MSG_LEN] = TEST_MSG_DATA;
    byte result = CAN.sendMsgBuf(TEST_MSG_ID, 0, TEST_MSG_LEN, testData);
    
    if (result == CAN_OK) {
        Serial.println("Test message sent successfully");
        canInitialized = true;
        canError = false;
    } else {
        Serial.printf("Failed to send test message, error code: %d\n", result);
        canError = true;
    }
}

void sendCANMessage() {
    if (!canInitialized) {
        Serial.println("Cannot send: CAN not initialized");
        return;
    }

    // Prepare Pylontech-compatible CAN message
    uint8_t canData[8] = {0};
    
    // Pack voltage (mV)
    uint16_t voltage = bmsData.voltage * 1000;
    canData[0] = voltage >> 8;
    canData[1] = voltage & 0xFF;
    
    // Pack current (mA)
    int16_t current = bmsData.current * 1000;
    canData[2] = current >> 8;
    canData[3] = current & 0xFF;
    
    // Pack SOC (0-100%)
    canData[4] = bmsData.soc;
    
    // Pack temperature
    canData[5] = bmsData.temperature + 40;
    
    // Pack status flags
    canData[6] = (bmsData.chargeEnabled ? 0x01 : 0) | 
                 (bmsData.dischargeEnabled ? 0x02 : 0) |
                 (bmsData.faultFlags ? 0x04 : 0);
    
    // Send CAN message with retry
    int retries = 3;
    bool sendSuccess = false;
    
    while (retries > 0 && !sendSuccess) {
        byte sendResult = CAN.sendMsgBuf(config.canId, 0, 8, canData);
        if (sendResult == CAN_OK) {
            sendSuccess = true;
            canActive = true;
            lastCanSend = millis();
            break;
        }
        retries--;
        if (retries > 0) {
            delay(10); // Short delay before retry
        }
    }

    if (!sendSuccess) {
        Serial.println("Failed to send CAN message after retries");
        canActive = false;
        canError = true;
    }
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
        Serial.println("Config saved to LittleFS.");
    } else {
        Serial.println("Failed to open config file for writing!");
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
                Serial.println("Config loaded from LittleFS.");
            } else {
                Serial.println("Failed to parse config file!");
            }
            file.close();
        } else {
            Serial.println("Failed to open config file for reading!");
        }
    } else {
        Serial.println("Config file does not exist, using defaults.");
    }
}

void setupWebServer() {
    Serial.println("Preparing to start web server...");
    delay(1000); // Give WiFi stack time to settle
    Serial.println("Starting AsyncWebServer on port 80...");
    
    // Register API endpoints FIRST
    server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest *request) {
        StaticJsonDocument<256> doc;
        doc["bmsStatus"] = (lastBMSStatus == BMS_OK) ? "OK" : (lastBMSStatus == BMS_TIMEOUT ? "Timeout" : "Error");
        doc["voltage"] = bmsData.voltage;
        doc["current"] = bmsData.current;
        doc["soc"] = bmsData.soc;
        doc["temperature"] = bmsData.temperature;
        doc["chargeEnabled"] = bmsData.chargeEnabled;
        doc["dischargeEnabled"] = bmsData.dischargeEnabled;
        doc["canActive"] = canActive;
        doc["canError"] = canError;
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

    server.on("/api/config", HTTP_POST, [](AsyncWebServerRequest *request){}, NULL,
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

    // Robust ESP32 OTA firmware update handler
    server.on("/update", HTTP_POST, [](AsyncWebServerRequest *request){
        bool shouldReboot = !Update.hasError();
        AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", shouldReboot ? "OK" : "FAIL");
        response->addHeader("Connection", "close");
        request->send(response);
        if (shouldReboot) {
            Serial.println("OTA update successful, rebooting...");
            delay(1000);
            ESP.restart();
        } else {
            Serial.println("OTA update failed.");
        }
    }, [](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final){
        if (!index){
            Serial.printf("OTA Update Start: %s\n", filename.c_str());
            if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { // start with max available size
                Update.printError(Serial);
            }
        }
        if (Update.write(data, len) != len) {
            Update.printError(Serial);
        }
        if (final) {
            if (Update.end(true)) {
                Serial.printf("OTA Update Success: %u bytes\n", index+len);
            } else {
                Update.printError(Serial);
            }
        }
    });

    // Serve static files from LittleFS
    server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");
    
    server.begin();
    Serial.println("Web server started");
} 