# 🌱 Smart Plant Monitor

An automated plant monitoring and irrigation system using ESP32, Blynk, and various sensors.

## 🛠 Features
- DHT11 temperature and humidity sensor
- Soil moisture monitoring
- Ultrasonic sensor for water tank level
- Automatic and manual pump control
- LCD display
- Buzzer and LED indicators
- Blynk IoT dashboard

## 🧰 Hardware Components
- ESP32
- DHT11 Sensor
- Soil Moisture Sensor
- Ultrasonic Sensor (HC-SR04)
- Relay Module
- Buzzer
- 3 LEDs (Red, Yellow, Green)
- LCD I2C (16x2)
- Push Button
- Power Supply

## 🔧 Software Tools
- Arduino IDE
- Blynk IoT
- C++ (Arduino framework)
- Libraries: WiFi.h, DHT.h, LiquidCrystal_I2C.h, BlynkSimpleEsp32.h


## 📋 How It Works
- The ESP32 collects sensor data and displays it on the LCD and Blynk dashboard.
- Based on thresholds, it automatically controls a water pump.
- Manual control is available via Blynk or a physical push button.
- Alerts are sent if the tank level is low.

## 🧠 Logic
- Auto watering if:
  - Soil is dry
  - Temperature is high and soil is slightly dry
  - Humidity is low and soil is slightly dry
- Pump won't run if water level is too low.

## 🔐 Security
Make sure to keep your BLYNK_AUTH_TOKEN secret when sharing the code.
