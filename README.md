| Supported Targets | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-C5 | ESP32-C6 | ESP32-C61 | ESP32-H2 | ESP32-H21 | ESP32-H4 | ESP32-P4 | ESP32-S2 | ESP32-S3 | Linux |
| ----------------- | ----- | -------- | -------- | -------- | -------- | --------- | -------- | --------- | -------- | -------- | -------- | -------- | ----- |

# Interactive MQTT Sensor (V182 Sensor Variant)

ESP-IDF project for an interactive MQTT sensor node supporting the **V182** digital temperature and humidity sensor (DHT11 / DHT22 / AM2302 module with 3-pin interface).

## Overview

This repository contains the "Interactive MQTT Sensor" firmware for ESP-IDF adapted for the **V182** sensor module (3-pin: `+`, `out`, `-`). The firmware reads temperature and relative humidity over a single-wire digital protocol and exposes their values via MQTT on demand.

## Features

- Reads temperature and humidity from the 3-pin **V182** sensor (auto-detects DHT11 and DHT22/AM2302 formats).
- Includes read caching (2 seconds) to avoid sensor rate-limiting when requesting temperature and humidity sequentially.
- Connects to Wi‑Fi and an MQTT broker.
- Interactive bootstrap via serial console to enter Wi‑Fi and MQTT settings, saved to NVS.
- Reset button clears saved configuration at boot (GPIO configured by `CONFIG_RESET_BUTTON_GPIO`).
- Subscribes to metric request topics under `Metrics/<metric>/<device_id>` and replies on the same topic with JSON payloads.
- Supports OTA update over HTTPS, triggered by MQTT command.

## Hardware Wiring

The **V182** sensor module has 3 pins:

| V182 Pin | ESP32-S3 Connection | Description |
| :--- | :--- | :--- |
| **`+`** | **3.3V** (or 5V) | Power supply |
| **`out`** | **GPIO 8** | Single-wire digital data signal |
| **`-`** | **GND** | Ground |

*Reset button:* GPIO 4 (active low / pull-to-GND to reset configuration).

## MQTT Topics

- Request metric values by publishing to: `Metrics/<metric>/<device_id>`
	- Example requests:
		- `Metrics/Temp/<device_id>` with payload `get` — request temperature in °C
		- `Metrics/HUMIDITY/<device_id>` — request relative humidity in %
		- (Note: requests for `PRESSURE` or `ALTITUDE` return `"not_supported"`)
- The device subscribes to `Metrics/+/<device_id>` and replies on the same topic.
- Request payload should be `get` (or empty/`1` for compatibility). JSON payloads are treated as responses and ignored to avoid MQTT feedback loops.
- Response payloads are JSON objects with `type` and `data` fields:
  - Success: `{"type":"Temp","data":23.5}` or `{"type":"HUMIDITY","data":45.2}`
  - Error: `{"type":"Temp","data":"read_error"}`
  - Unsupported: `{"type":"PRESSURE","data":"not_supported"}`

## OTA Over MQTT

- OTA command topic: `OTA/update/<device_id>`
- OTA status topic: `OTA/status/<device_id>`
- Payload formats supported for OTA command:
	- raw HTTPS URL string, example: `https://example.com/firmware.bin`
	- JSON object, example: `{"url":"https://example.com/firmware.bin"}`

Notes:
- OTA download is allowed only from `https://` URLs.
- The device publishes status transitions (`accepted`, `downloading`, `applied`, `failed`, `busy`) to `OTA/status/<device_id>`.
- On successful update, the device reboots automatically.

## Interactive Setup and Configuration

- On first boot (or after clearing NVS by holding the reset button on GPIO4 for at least 1 second at boot), the device enters an interactive console setup where you must provide:
	- Wi‑Fi SSID and password
	- Broker URI (e.g. `mqtt://192.168.1.10:1883` or `wss://broker.example.com:443/mqtt`)
	- MQTT client id, username, password, and device id
- The settings are stored in NVS under namespace `appcfg` so the device reconnects automatically on next boot.

## Build and Flash

Build and flash using the ESP-IDF tooling from the project root:

```bash
idf.py build
idf.py -p PORT flash
idf.py -p PORT monitor
```

Replace `PORT` with your serial port (example: `COM5` on Windows).

## Project files

- [CMakeLists.txt](CMakeLists.txt#L1) — top-level build config
- [main/interactive_mqtt_sensor_main.c](main/interactive_mqtt_sensor_main.c#L1) — firmware entrypoint
- [main/sensors.c](main/sensors.c#L1) — V182 (DHT11/DHT22) single-wire driver
- [main/connectivity.c](main/connectivity.c#L1) — Wi-Fi, MQTT, and OTA handling
