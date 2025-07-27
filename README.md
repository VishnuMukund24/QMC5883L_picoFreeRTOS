# QMC5883L_picoFreeRTOS
A FreeRTOS-based driver and application for the QMC5883L 3-axis magnetometer sensor on Raspberry Pi Pico.

## Overview
This project provides a complete implementation for interfacing with the QMC5883L magnetometer sensor using the Raspberry Pi Pico microcontroller running FreeRTOS. The QMC5883L is a 3-axis magnetic sensor commonly found on GY-271 compass modules, capable of measuring magnetic field strength and direction.

## Features

- FreeRTOS Integration: Multi-tasking support with real-time sensor data acquisition
- I2C Communication: Efficient communication with QMC5883L sensor
- 3-Axis Magnetometer: Read X, Y, Z magnetic field values
- Compass Functionality: Calculate heading/azimuth from magnetic readings
- Calibration Support: Hard and soft iron calibration capabilities
- Non-blocking Operations: Asynchronous sensor reading using FreeRTOS tasks
- Error Handling: Robust error detection and recovery mechanisms

## Hardware Requirements

Raspberry Pi Pico (RP2040-based board)
QMC5883L Magnetometer (GY-271 module or equivalent)
Connecting Wires
Breadboard (optional)

## Pin Connections
| QMC5883L (GY-271) | Raspberry Pi Pico |
|-------------------|-------------------|
| VCC               | 3.3V (Pin 36)    |
| GND               | GND (Pin 38)     |
| SCL               | GP1 (Pin 2)      |
| SDA               | GP0 (Pin 1)      |

Note: Default I2C pins can be configured in the source code

## Software Dependencies

- Raspberry Pi Pico SDK
- FreeRTOS-Kernel (included as submodule)
- CMake (build system)
- GCC ARM Embedded Toolchain

## Getting Started
### 1. Clone the Repository
```bash
git clone --recursive https://github.com/VishnuMukund24/QMC5883L_picoFreeRTOS.git
cd QMC5883L_picoFreeRTOS
```
### 2. Install Dependencies
Make sure you have the Raspberry Pi Pico SDK installed and the PICO_SDK_PATH environment variable set:
```bash
export PICO_SDK_PATH=/path/to/pico-sdk
```
### 3. Build the project
```bash
mkdir build
cd build
cmake ..
make
```
### 4. Flash to pico
- Hold the BOOTSEL button while connecting the Pico to your computer
- Copy the generated QMC5883L_picoFreeRTOS.uf2 file to the Pico's mass storage device
