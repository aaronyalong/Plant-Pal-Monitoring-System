# Plant Monitoring System

## Overview
This project is a plant monitoring system using an ESP32-based TTGO board. It collects environmental data such as temperature, humidity, soil moisture, and light exposure. The system displays real-time data on an LCD and connects to a cloud server for remote monitoring.

## Features
- **Temperature & Humidity Monitoring:** Uses the DHT20 sensor to measure environmental conditions.
- **Soil Moisture Detection:** Detects soil moisture levels and suggests watering based on predefined thresholds.
- **Light Exposure Measurement:** Monitors UV/light levels and provides alerts if exposure is too low or too high.
- **LCD Display:** Displays real-time sensor readings and alerts.
- **WiFi Connectivity:** Connects to a cloud server for data logging and remote access.
- **Predictive Watering Alerts:** Uses past humidity trends to predict when watering is needed.

## Hardware Components
- **ESP32 TTGO Board** (Main controller)
- **DHT20 Sensor** (Temperature & Humidity)
- **Soil Moisture Sensor**
- **UV/Light Sensor**
- **TFT LCD Display**
- **LED Indicators** (Normal & Alert)
- **Resistors & Connecting Wires**

## Pin Configuration
| Component               | Pin on ESP32 |
|-------------------------|-------------|
| UV Sensor              | GPIO 27     |
| Normal LED             | GPIO 26     |
| Alert LED              | GPIO 25     |
| Soil Moisture Sensor   | GPIO 33     |
| Soil Sensor Power      | GPIO 32     |
| I2C SDA (DHT20 & LCD)  | GPIO 21     |
| I2C SCL (DHT20 & LCD)  | GPIO 22     |

## Setup Instructions
1. **Install Required Libraries:** Ensure you have the following libraries in the Arduino IDE:
   - `WiFi.h`
   - `HttpClient.h`
   - `DHT20.h`
   - `TFT_eSPI.h`
   - `freertos/FreeRTOS.h`
   - `esp_system.h`
2. **Configure WiFi Credentials:** Update the `ssid` and `pass` variables with your network credentials.
3. **Upload Code to ESP32:** Compile and upload the provided code to your ESP32 board.
4. **Monitor Serial Output:** Open the Serial Monitor at `9600 baud` to view sensor readings and system logs.

## WiFi & Cloud Integration
- The system connects to a cloud server (`PLANTPAL.org`) for remote data logging.
- Sends temperature, humidity, light, and soil moisture data over HTTP.
- Retrieves responses and logs cloud data in real-time.

## Future Improvements
- Implement MQTT for real-time cloud updates.
- Add a mobile app for remote monitoring and notifications.
- Integrate a water pump for automatic irrigation.
- Improve security by encrypting WiFi credentials.

## Author
Developed as part of an IoT project using ESP32 and environmental sensors.

