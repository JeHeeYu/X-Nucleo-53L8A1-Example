# X-Nucleo-53L8A1-Example

## Introduction
This project is an example code for testing the TOF (Time of Flight) sensor using the STMicroelectronics X-NUCLEO-53L8A1 module. The code utilizes the VL53L8CX sensor to perform distance measurement and object recognition.

<br>

## Requirements

### Hardware
- Nucleo development board (STM32F401RE)
- X-NUCLEO-53L8A1 TOF module

### Software
- STM32CubeIDE
- VL53L8CX API library

[API Download Link](https://www.st.com/en/embedded-software/stsw-img040.html)

<br>

## Usage
1. After building the project, connect the Nucleo board to power.
2. Monitor the serial output to view the TOF sensor measurement results.
3. The sensor will measure distances and output results whenever it detects an object.

<br>

## Features
- Distance Measurement: Measures the distance to objects using the VL53L8CX sensor.
- Calibration: Uses calibration data to optimize sensor performance.

<br>

## Caution
- The reflectivity of the measurement environment can vary, so appropriate settings should be applied through calibration.
- Distance measurements may be influenced by environmental factors (lighting, color, etc.).
