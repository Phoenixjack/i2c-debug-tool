# i2c-debug-tool

An Arduino-based serial shell for scanning, identifying, reading, dumping, and writing I2C devices during hardware bring-up and troubleshooting.

This tool is intended for bench testing sensors, displays, RTCs, GPIO expanders, EEPROMs, and other common I2C peripherals before writing a full driver or integrating the device into a larger project.

## Features

* Scan the I2C bus for connected 7-bit devices
* Match detected addresses against a small built-in device database
* Confirm some devices by checking known chip ID registers
* Dump a register range from a selected I2C address
* Read a single register
* Write a value to a specific register
* Add unknown devices to EEPROM for later identification

## Target Platform

This sketch is written for Arduino-compatible boards using the standard `Wire` library.

Current includes:

```cpp
#include <Wire.h>
#include <EEPROM.h>
#include <avr/pgmspace.h>
```

Because it uses AVR-specific `PROGMEM`, it is best treated as an Arduino/AVR sketch unless modified for other platforms.

## Hardware Requirements

* Arduino-compatible board
* One or more I2C devices
* Correct pull-up resistors on SDA/SCL, if not already present on the board/module
* Common ground between the controller and device under test
* Serial monitor set to `115200` baud

Default pins use the board-defined I2C pins:

```cpp
const uint8_t I2C_SDA_PIN = SDA;
const uint8_t I2C_SCL_PIN = SCL;
```

## Serial Settings

Open the Arduino Serial Monitor or another serial terminal with:

```text
Baud: 115200
Line ending: Newline
```

On startup, the sketch prints:

```text
I2C Debug Shell Ready
Type 'help' for commands.
>
```

## Commands

All numeric values are entered as hexadecimal.

| Command                     | Description                                  |
| --------------------------- | -------------------------------------------- |
| `help`                      | Show available commands                      |
| `scan`                      | Scan the I2C bus and identify likely devices |
| `dump <addr> [start] [end]` | Dump a register range from a device          |
| `read <addr> <reg>`         | Read one register                            |
| `write <addr> <reg> <val>`  | Write one byte to one register               |

## Examples

Scan the bus:

```text
scan
```

Read register `0x75` from device `0x68`:

```text
read 0x68 0x75
```

Dump registers `0x00` through `0x7F` from device `0x68`:

```text
dump 0x68 0x00 0x7F
```

Write value `0x01` to register `0x10` on device `0x40`:

```text
write 0x40 0x10 0x01
```

## Built-In Device Matching

The sketch includes a small lookup table for common I2C devices, including examples such as:

* MPU-6050
* MPU-9250 / ICM-20948
* BME280 / BMP280
* BME680
* HTU21D / Si7021
* BH1750
* TSL2561 / APDS-9960
* LIS3DH
* LIS3MDL
* LSM6DS3 / LSM6DS33
* ADXL345
* MCP23008 / MCP23017
* PCF8574A LCD backpacks
* ADS111x / TMP102
* SSD1306 OLED displays
* DS1307 / DS3231 RTCs
* 24Cxx EEPROMs

Some devices are identified only by address. Others are checked against a known chip ID register when available.

## EEPROM Device Entries

When the scanner finds an address that is not in the built-in device list, the sketch can prompt you to save a custom device entry to EEPROM.

Each saved entry can include:

* I2C address
* Device name
* Optional chip ID register
* Expected chip ID value
* ID mask

This is useful for recurring bench work with custom boards, breakout modules, or less common I2C parts.

## Safety Notes

The `write` command can change device configuration registers. On some devices, this can affect calibration, operating mode, EEPROM contents, outputs, or power behavior.

Use `write` carefully. When in doubt, read the device datasheet first.

## Limitations

* Uses 7-bit I2C addresses
* Register reads are byte-oriented
* Bulk dump reads are chunked
* Some devices require special command sequences and may not behave like simple register-mapped devices
* AVR-specific `PROGMEM` usage may require changes for ESP32, RP2040, SAMD, or other platforms
* Device identification is best-effort and should be verified against the datasheet

## Status

Functional bench utility / work-in-progress.

This is intended as a practical troubleshooting tool, not a polished universal I2C driver framework.

## License

This project is released under the MIT License. Use it freely, modify it, share it, or adapt it for your own projects.

No warranty is provided. Use at your own risk.
