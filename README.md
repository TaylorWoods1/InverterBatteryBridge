# Inverter Battery Bridge

A robust ESP32-based bridge that connects Pylontech battery systems to Sungrow inverters via CAN bus. This project provides reliable communication between battery management systems and inverters, with features for monitoring, configuration, and error recovery.

## Features

- CAN bus communication with Pylontech batteries
- Web interface for monitoring and configuration
- Prometheus-style metrics endpoint
- Automatic error recovery mechanisms
- Watchdog timer for system stability
- WiFi configuration via WiFiManager
- Power supply monitoring
- LED status indicators

## Hardware Requirements

- ESP32 development board
- MCP2515 CAN controller
- CAN transceiver (e.g., TJA1050)
- Power supply (12V recommended)
- Level shifter (if needed for 3.3V/5V conversion)

## Pin Configuration

```
ESP32 Pin    ->    MCP2515 Pin
VSPI_MISO    ->    SO
VSPI_MOSI    ->    SI
VSPI_SCLK    ->    SCK
CAN_CS       ->    CS
CAN_INT      ->    INT
CAN_RESET    ->    RST
POWER_MONITOR_PIN -> Voltage divider
```

## Installation

1. Clone this repository:
   ```bash
   git clone https://github.com/yourusername/InverterBatteryBridge.git
   cd InverterBatteryBridge
   ```

2. Install PlatformIO:
   ```bash
   pip install platformio
   ```

3. Build and upload:
   ```bash
   pio run -t upload
   ```

## Configuration

The bridge can be configured through:

1. Web interface (http://<device-ip>)
2. Configuration file in LittleFS
3. WiFiManager portal (first boot)

### Default Settings

- CAN Speed: 500 kbps
- CAN ID: 0x4210
- Battery Capacity: 17.4 kWh
- Module Count: 7
- Poll Interval: 1000 ms

## Monitoring

### Web Interface

Access the web interface at `http://<device-ip>` to:
- View real-time status
- Configure settings
- Monitor error counters
- View system metrics

### Metrics Endpoint

Access Prometheus-style metrics at `http://<device-ip>/metrics`:
- CAN error counters
- WiFi signal strength
- Power supply voltage
- System uptime

## Error Recovery

The system includes automatic recovery mechanisms for:
- CAN bus errors
- WiFi disconnections
- System hangs (watchdog timer)

Recovery attempts are limited to prevent infinite loops, with a full system reset as a last resort.

## Development

### Project Structure

```
InverterBatteryBridge/
├── src/
│   └── main.cpp
├── include/
│   └── config.h
├── data/
│   └── index.html
├── platformio.ini
└── README.md
```

### Building

```bash
# Build
pio run

# Upload
pio run -t upload

# Monitor
pio device monitor
```

## Troubleshooting

1. CAN Bus Issues:
   - Check physical connections
   - Verify CAN termination
   - Monitor error counters via web interface

2. WiFi Issues:
   - Hold BOOT button during power-up to enter WiFiManager
   - Check WiFi credentials
   - Monitor signal strength

3. Power Issues:
   - Verify voltage at POWER_MONITOR_PIN
   - Check power supply stability
   - Monitor voltage via metrics endpoint

## Contributing

1. Fork the repository
2. Create a feature branch
3. Commit your changes
4. Push to the branch
5. Create a Pull Request

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

- MCP_CAN library
- ESPAsyncWebServer
- WiFiManager
- PlatformIO 