| Supported Targets | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-C5 | ESP32-C6 | ESP32-C61 | ESP32-H2 | ESP32-H21 | ESP32-H4 | ESP32-P4 | ESP32-S2 | ESP32-S3 | Linux |
| ----------------- | ----- | -------- | -------- | -------- | -------- | --------- | -------- | --------- | -------- | -------- | -------- | -------- | ----- |

# Interactive MQTT Sensor

ESP-IDF project for an interactive MQTT sensor node.

(See the README.md file in the upper level 'examples' directory for more information about examples.)

## Overview

This repository contains the "Interactive MQTT Sensor" firmware for ESP-IDF. The firmware reads environmental sensors over I2C and exposes their values via MQTT on demand. It is intended for development boards with an AHT20 (temperature & humidity) and a BMP280 (pressure) connected on the I2C bus.

## Features

- Reads temperature and humidity from AHT20.
- Reads pressure (and computes altitude) from BMP280.
- Connects to Wi‑Fi and an MQTT broker.
- Interactive bootstrap via serial console to enter Wi‑Fi and MQTT settings, saved to NVS.
- Reset button clears saved configuration at boot (GPIO configured by `CONFIG_RESET_BUTTON_GPIO`).
- Subscribes to metric request topics under `Metrics/<metric>/<device_id>` and replies on the same topic with JSON payloads.

## MQTT Topics

- Request metric values by publishing to: `Metrics/<metric>/<device_id>`
	- Example requests:
		- `Metrics/Temp/<device_id>` with payload `get` — request temperature
		- `Metrics/HUMIDITY/<device_id>` — request humidity
		- `Metrics/PRESSURE/<device_id>` — request pressure
		- `Metrics/ALTITUDE/<device_id>` — request altitude (computed from pressure)
- The device subscribes to `Metrics/+/<device_id>` and replies on the same topic.
- Request payload should be `get` (or empty/`1` for compatibility). JSON payloads are treated as responses and ignored to avoid MQTT feedback loops.
- Response payloads are JSON objects with `type` and `data` fields, where `data` contains the numeric value or the string `"read_error"` on failure.

## Interactive Setup and Configuration

- On first boot (or after clearing NVS by holding the reset button at boot), the device enters an interactive console setup where you must provide:
	- Wi‑Fi SSID and password
	- Broker URI (e.g. `mqtt://192.168.1.10:1883` or `wss://broker.example.com:443/mqtt`)
	- MQTT client id and device id
- The settings are stored in NVS under namespace `appcfg` so the device reconnects automatically on next boot.

## Hardware

- I2C SDA: GPIO 8
- I2C SCL: GPIO 9
- Reset button: GPIO defined by `CONFIG_RESET_BUTTON_GPIO` (default in project config)

Refer to the firmware entrypoint and implementation: [main/interactive_mqtt_sensor_main.c](main/interactive_mqtt_sensor_main.c#L1-L999).

## Build and Flash

Build and flash using the ESP-IDF tooling from the project root:

```bash
idf.py build
idf.py -p PORT flash
idf.py -p PORT monitor
```

Replace `PORT` with your serial port (example: `COM3` on Windows).

## Project files

- [CMakeLists.txt](CMakeLists.txt#L1) — top-level build config
- [main/interactive_mqtt_sensor_main.c](main/interactive_mqtt_sensor_main.c#L1) — firmware source

If you need more details on ESP-IDF project structure and building, see the ESP-IDF Programming Guide: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/

## Troubleshooting

* Program upload failure

	* Hardware connection is not correct: run `idf.py -p PORT monitor`, and reboot your board to see if there are any output logs.
	* The baud rate for downloading is too high: lower your baud rate in the `menuconfig` menu, and try again.

## Technical support and feedback

Please use the following feedback channels:

* For technical queries, go to the [esp32.com](https://esp32.com/) forum
* For a feature request or bug report, create a [GitHub issue](https://github.com/espressif/esp-idf/issues)

We will get back to you as soon as possible.

