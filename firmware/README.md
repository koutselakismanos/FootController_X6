# FootController_X6 Firmware

This directory contains the ESP-IDF firmware for the FootController_X6 project.

## Overview

This firmware runs on the ESP32 and is responsible for:

*   Reading the state of the 6 connected footswitches (GPIOs 4, 5, 6, 38, 47, 48) with debouncing.
*   Managing multiple configuration layers (up to 5).
*   Detecting switch hold actions (momentary layer, toggle layer, send MIDI).
*   Handling optional toggle (latching) behavior for switches.
*   Generating USB MIDI Class Compliant messages based on switch actions and configuration.
*   Receiving configuration updates in JSON format via the USB CDC Serial interface.
*   Saving/loading configuration to/from Non-Volatile Storage (NVS).

For complete project details, hardware information, and instructions on how to configure the device using JSON, please refer to the main [README.md](../../README.md) in the project root directory.

## How to Use (Build and Flash)

### Prerequisites

*   ESP-IDF environment setup (refer to official Espressif documentation).

### Build and Flash

1.  Ensure you are in the `firmware/` directory.
2.  Configure the project if necessary (e.g., select ESP32 target):
    ```bash
    idf.py menuconfig
    ```
3.  Build the firmware:
    ```bash
    idf.py build
    ```
4.  Flash the firmware to the ESP32 (replace `PORT` with your device's serial port):
    ```bash
    idf.py -p PORT flash
    ```
5.  Monitor the output (useful for debugging and seeing configuration confirmation):
    ```bash
    idf.py -p PORT monitor
    ```
    (To exit the serial monitor, type `Ctrl-]`)

See the ESP-IDF Getting Started Guide for full steps to configure and use ESP-IDF to build projects.
