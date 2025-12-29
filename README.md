# STM32 Environmental Monitoring Station

A comprehensive environmental monitoring system built with STM32L152RE microcontroller that monitors temperature and humidity, provides visual feedback, controls a fan for climate management, and logs data over UART.

## Features

- Temperature & Humidity Monitoring - Real-time sensing using DHT22 sensor
- OLED Display - Multi-screen interface showing current readings, historical graphs, and fan control
- Visual Alerts - RGB LED with blinking patterns for alert conditions
- Automatic Fan Control - Climate-responsive fan activation
- Data Logging - JSON-formatted data transmission over UART for remote logging into a database
- User Interface - Push-button navigation and alert override functionality
- Historical Data - Store and display temperature/humidity trends

## Hardware Components

- STM32L152RE - Low-power ARM Cortex-M3 microcontroller
- DHT22 - Temperature and humidity sensor (±0.5°C, ±2-5% RH accuracy)
- 128x64 AdaFruit OLED Display - 128x64 pixel I2C display
- 4-Pin RGB LED - Common cathode RGB LED for status indication
- 5V DC Fan - Motor control for climate management
- Push Buttons - Two buttons for navigation and alert control

## Software Architecture

### Dependencies
- SSD1306 Library - [afiskon/stm32-ssd1306](https -//github.com/afiskon/stm32-ssd1306/tree/master) for OLED display control

## Operating Modes

### Alert Conditions
- Temperature Alert - Triggered when temperature > 25°C
- Humidity Alert - Triggered when humidity > 50%
- Combined Alert - Both temperature and humidity exceed thresholds

### LED Status Indicators
- Green Solid - Normal conditions
- Red Blinking - Alert condition active
- Off - Alert override activated

### Display Screens
1. Current Readings (Screen 0) - Live temperature, humidity, and fan status
2. Temperature Graph (Screen 2) - Historical temperature trend
3. Humidity Graph (Screen 3) - Historical humidity trend  
4. Fan Control (Screen 4) - Fan status and speed information

### Button Controls
- Button PB5 - Navigate to next screen
- Button PB3 - Navigate to previous screen
- Both Buttons - Toggle LED alert override (during alert conditions only)

## Configuration

### Thresholds
```c
#define ALERT_TEMPERATURE_THRESHOLD 25.0f  // Celsius
#define ALERT_HUMIDITY_THRESHOLD 50.0f     // Percent RH
```

### Timing Intervals
```c
#define SERIAL_INTERVAL 30000           // Data transmission (30 seconds)
#define DATA_COLLECTION_INTERVAL 30000  // Sensor sampling (30 seconds) 
#define BLINK_INTERVAL 500              // LED blink rate (500ms)
```

## Data Format

### UART Output
Data is transmitted as JSON over UART at 115200 baud -

```json
{
  "timestamp" - 12345678,
  "temperature" - 23.45,
  "humidity" - 45.67,
  "fan_status" - 1,
  "alert_condition" - 1
}
```

### Alert Condition Values
- `0` - No alert
- `1` - Temperature alert only
- `2` - Humidity alert only  
- `3` - Both temperature and humidity alerts

## License

This project is provided under the MIT License. See LICENSE file for details.
