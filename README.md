# Internship Training Report: Open IoT and LoRaWAN

This repository contains the documentation and resources related to my ICFOSS Internship (Dec 2024 - Apr 2025) at the International Centre for Free and Open Source Solutions. The focus of the internship was on Open IoT, particularly using ULP LoRa boards, LoRaWAN communication, and sensor integrations.

---

## Table of Contents

1. [Project Overview](#project-overview)
2. [Hardware Components](#hardware-components)
3. [Software Tools](#software-tools)
4. [Experiments and Results](#experiments-and-results)
5. [How to Use This Repository](#how-to-use-this-repository)
6. [Acknowledgements](#acknowledgements)

---

## Project Overview

This internship involved:
- Exploring the capabilities of ULP LoRa (Ultra Low Power LoRa) boards for IoT applications.
- Interfacing various sensors like BMP180, DHT11, and soil moisture sensors.
- Utilizing communication protocols such as SPI, I2C, and UART.
- Implementing LoRaWAN communication for long-range, low-power wireless data transmission.
- Visualizing sensor data using tools like Grafana and ChirpStack.

### Objectives:
1. Build hands-on expertise with ULP LoRa boards.
2. Integrate and monitor sensor data using LoRaWAN technology.
3. Document experiments for educational purposes.

---

## Hardware Components

The following components were used during the internship:

- **ULP LoRa Board**:
  - Microcontroller: ATmega328P (3.3V, 8MHz)
  - RFM95W LoRa module
  - Digital Pins: 12 (D2-D13)
  - Analog Pins: 3 (A0-A2)

- **Sensors**:
  - BMP180 Atmospheric Pressure and Temperature Sensor
  - BME280 Environmental Sensor
  - DHT11 Temperature and Humidity Sensor
  - Soil Moisture Sensor

- **Additional Components**:
  - LEDs, resistors, and push buttons for logic gate simulations
  - FTDI FT232RL USB to Serial Adapter
  - Servo motors

---

## Software Tools

### 1. Arduino IDE
- Used for programming and uploading sketches to the ULP LoRa board.

### 2. ChirpStack
- Open-source LoRaWAN Network Server for device communication.

### 3. InfluxDB
- Time-series database for storing sensor data.

### 4. Grafana
- Visualization platform for analyzing sensor data trends.

### 5. Libraries
- MCCI LoRaWAN LMIC Library
- BMP180TwoWire Library
- DHT Sensor Library

---

## Experiments and Results

### GPIO Peripherals
- **LED Blinking**: Controlled an LED via ULP LoRa board GPIO pins.
- **Logic Gate Simulations**: Implemented AND, OR, XOR, NAND, and NOR gates using LEDs and buttons.

### Sensor Integrations
- **BMP180**: Measured temperature and pressure; controlled LEDs based on thresholds.
- **Soil Moisture Sensor**: Monitored and visualized soil moisture levels on Grafana.
- **DHT11**: Captured temperature and humidity data for IoT visualization.

### Communication Protocols
- SPI, I2C, and UART were implemented for sensor communication and data transmission.

---

## How to Use This Repository

1. **Documentation**:
   - Refer to the `docs/` folder for detailed experimental write-ups.
2. **Code**:
   - The `code/` folder contains Arduino sketches for the experiments.
3. **Diagrams and Schematics**:
   - Visit the `media/` folder for circuit diagrams and sensor setup images.

---

## Acknowledgements

Special thanks to ICFOSS for providing this internship opportunity and the resources to explore LoRaWAN and IoT applications. The guidance from mentors and support from team members were invaluable in completing the project successfully.

---


