# SEP600_Project

**FRDM-K64F Sensor Dashboard with AI Prediction & Digital Twin LED Control**

## Overview

This project implements a real-time IoT sensor monitoring system using a **FRDM-K64F** microcontroller and an **ESP32** WiFi module. The FRDM-K64F reads sensor data and controls an external LED, while the ESP32 hosts a web-based dashboard accessible over WiFi. The system features AI-based prediction using linear regression and a bidirectional Digital Twin for remote LED control.

## System Architecture

```
┌─────────────────────┐       UART (9600 baud)       ┌─────────────────────┐
│     FRDM-K64F       │◄────────────────────────────►│       ESP32         │
│                     │   TX/RX (PTC16, PTC17)        │                     │
│  • LDR Sensor (A0)  │                               │  • WiFi Web Server  │
│  • Temp Sensor (A5) │  Sensor data ──────────►      │  • JSON API         │
│  • External LED (A1)│  ◄────────── LED commands     │  • AI Dashboard     │
└─────────────────────┘                               └─────────────────────┘
                                                              │
                                                              ▼
                                                      ┌─────────────┐
                                                      │  Web Browser │
                                                      │  Dashboard   │
                                                      └─────────────┘
```

## Hardware Components

| Component | Pin / Connection | Description |
|-----------|-----------------|-------------|
| LDR (Light Sensor) | A0 (PTB2, ADC0_SE12) | Analog light level (0–4095) |
| LM35DZ Temp Sensor | A5 (PTC10, ADC1_SE6b) | Analog temperature (10mV/°C) |
| External Green LED | A1 (PTB3, GPIO) | Digital output, controlled via dashboard or auto |
| ESP32 UART TX | PTC17 (UART3 TX) | FRDM transmits sensor data to ESP32 |
| ESP32 UART RX | PTC16 (UART3 RX) | FRDM receives LED commands from ESP32 |

## Files

| File | Target | Description |
|------|--------|-------------|
| `SEDProject.c` | FRDM-K64F | Main firmware — reads sensors, controls LED, communicates with ESP32 via UART3 using interrupt-driven RX |
| `esp32.ino` | ESP32 | WiFi web server — hosts the dashboard, serves JSON API, relays LED commands to FRDM via UART |

## Features

### Sensor Monitoring
- **LDR Light Level**: Real-time ADC reading displayed on dashboard
- **Temperature**: LM35DZ analog sensor converted to °C

### LED Control Modes
- **AUTO Mode**: LED turns ON automatically when light level drops below threshold (LDR < 1000)
- **MANUAL Mode**: LED controlled remotely via the web dashboard toggle switch
- **Mode Switching**: Dashboard button to switch between AUTO and MANUAL modes

### Digital Twin (Bidirectional)
- **Dashboard → Physical**: Clicking the LED toggle sends a UART command (`CMD:LED_ON\n` / `CMD:LED_OFF\n`) from ESP32 to FRDM, which physically toggles the LED
- **Physical → Dashboard**: The FRDM continuously sends its current LED state and sensor readings to the ESP32, which updates the dashboard in real time

### AI Prediction (Linear Regression)
- **Light Prediction**: Uses the last 20 LDR readings to predict future light levels at 5s, 10s, and 30s intervals, and warns if the LED will change state
- **Temperature Prediction**: Predicts temperature trends and provides ventilation recommendations (open/close) based on threshold crossings (23°C–27°C)
- **Confidence Score**: R² value from linear regression indicates model reliability

## UART Communication Protocol

### FRDM → ESP32 (Sensor Data)
```
LDR:2021,TEMP:23.84,LED:OFF,MODE:AUTO\r\n
```

### ESP32 → FRDM (Commands)
```
CMD:LED_ON\n       → Switch to MANUAL mode, turn LED ON
CMD:LED_OFF\n      → Switch to MANUAL mode, turn LED OFF
CMD:AUTO\n         → Switch back to AUTO mode
```

## UART RX Implementation

The FRDM-K64F uses **interrupt-driven UART RX** with a 128-byte software ring buffer (`UART3_RX_TX_IRQHandler`). This ensures that every incoming byte from the ESP32 is captured immediately, even while the main loop is busy transmitting sensor data or reading ADCs. This approach is necessary because UART3 on the K64F has only a 1-byte hardware RX FIFO.

## How to Build & Flash

### FRDM-K64F
1. Open `SEDProject.c` in **MCUXpresso IDE**
2. Build the project with the NXP SDK for MK64F12
3. Flash to the FRDM-K64F board via USB

### ESP32
1. Open `esp32.ino` in **Arduino IDE**
2. Select board: **ESP32 Dev Module**
3. Update WiFi credentials in the file (`HOME_SSID` / `HOME_PASSWORD`)
4. Upload to the ESP32

### Accessing the Dashboard
1. Open the Arduino Serial Monitor (115200 baud) to find the ESP32's IP address
2. Navigate to `http://<ESP32_IP>` in a web browser

## Dashboard Preview

The web dashboard includes:
- Real-time sensor cards (LDR, Temperature, LED status)
- Digital Twin section with toggle switch and physical LED mirror
- AI Light Prediction module with trend analysis and recommendations
- AI Temperature Prediction module with ventilation recommendations
- Sensor log table with timestamped entries
