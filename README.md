# i2c-debug-tool

An Arduino-based serial shell for scanning, identifying, reading, dumping, and writing I2C devices during hardware bring-up and troubleshooting.

This tool is intended for bench testing sensors, displays, RTCs, GPIO expanders, EEPROMs, and other common I2C peripherals before writing a full driver or integrating the device into a larger project.

## Features

- Scan the I2C bus for connected 7-bit devices
- Match detected addresses against a small built-in device database
- Confirm some devices by checking known chip ID registers
- Dump a register range from a selected I2C address
- Read a single register
- Write a value to a specific register
- Add unknown devices to EEPROM for later identification

## Target Platform

This sketch is written for Arduino-compatible boards using the standard `Wire` library.

Current includes:

```cpp
#include <Wire.h>
#include <EEPROM.h>
#include <avr/pgmspace.h>
