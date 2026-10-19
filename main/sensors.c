#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sensors.h"

/*
 * Sensor: V182 (DHT11 / DHT22 / AM2302 single-wire digital temperature & humidity sensor)
 * Pinout:
 *   '+'   -> 3.3V (or 5V)
 *   'out' -> GPIO 8
 *   '-'   -> GND
 */
#define APP_DHT_DATA_GPIO 8

static const char *TAG = "v182_sensor";

static portMUX_TYPE s_dht_mux = portMUX_INITIALIZER_UNLOCKED;
static bool s_sensors_initialized = false;

// Caching: DHT sensors cannot be read faster than every 1-2 seconds.
// Caching avoids read errors when Temp and HUMIDITY are requested sequentially over MQTT.
static bool s_cache_valid = false;
static int64_t s_last_read_time = 0;
static float s_cached_temp = 0.0f;
static float s_cached_hum = 0.0f;

static int wait_level(gpio_num_t pin, int level, uint32_t timeout_us)
{
    int64_t start = esp_timer_get_time();
    while (gpio_get_level(pin) != level) {
        if ((esp_timer_get_time() - start) > timeout_us) {
            return -1;
        }
    }
    return (int)(esp_timer_get_time() - start);
}

static esp_err_t dht_read_raw(gpio_num_t pin, uint8_t data[5])
{
    memset(data, 0, 5);

    // 1. Send start signal: LOW for at least 18-20ms
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
    gpio_set_level(pin, 0);
    vTaskDelay(pdMS_TO_TICKS(20));

    // Pull HIGH for 30us before switching to input
    gpio_set_level(pin, 1);
    esp_rom_delay_us(30);

    // Switch to input with pullup enabled
    gpio_set_direction(pin, GPIO_MODE_INPUT);
    gpio_set_pull_mode(pin, GPIO_PULLUP_ONLY);

    // 2. Measure response and 40 data bits inside critical section to avoid FreeRTOS preemption
    esp_err_t result = ESP_OK;
    portENTER_CRITICAL(&s_dht_mux);

    // Wait for sensor response: LOW (~80us) -> HIGH (~80us) -> LOW (start of data)
    if (wait_level(pin, 0, 100) < 0 ||
        wait_level(pin, 1, 100) < 0 ||
        wait_level(pin, 0, 100) < 0) {
        result = ESP_ERR_TIMEOUT;
        goto cleanup;
    }

    // 3. Read 40 data bits
    for (int i = 0; i < 40; i++) {
        // Wait for bit start (LOW -> HIGH)
        if (wait_level(pin, 1, 100) < 0) {
            result = ESP_ERR_TIMEOUT;
            goto cleanup;
        }

        // Measure HIGH duration (HIGH -> LOW)
        int high_duration = wait_level(pin, 0, 120);
        if (high_duration < 0) {
            result = ESP_ERR_TIMEOUT;
            goto cleanup;
        }

        // '0' is ~26-28us, '1' is ~70us. Threshold is ~45us.
        if (high_duration > 45) {
            data[i / 8] |= (1 << (7 - (i % 8)));
        }
    }

cleanup:
    portEXIT_CRITICAL(&s_dht_mux);
    return result;
}

static bool dht_decode(const uint8_t data[5], float *temperature_c, float *humidity_rh)
{
    uint8_t sum = (data[0] + data[1] + data[2] + data[3]) & 0xFF;
    if (sum != data[4]) {
        ESP_LOGW(TAG, "V182 checksum mismatch: got 0x%02x, calculated 0x%02x (raw: %02x %02x %02x %02x %02x)",
                 data[4], sum, data[0], data[1], data[2], data[3], data[4]);
        return false;
    }

    // Decode as DHT22 / AM2302
    float dht22_h = (float)(((uint16_t)data[0] << 8) | data[1]) * 0.1f;
    float dht22_t = (float)(((uint16_t)(data[2] & 0x7F) << 8) | data[3]) * 0.1f;
    if (data[2] & 0x80) {
        dht22_t = -dht22_t;
    }

    // Decode as DHT11
    float dht11_h = (float)data[0] + (float)data[1] * 0.1f;
    float dht11_t = (float)(data[2] & 0x7F) + (float)data[3] * 0.1f;
    if (data[2] & 0x80) {
        dht11_t = -dht11_t;
    }

    // Auto-detect DHT22 vs DHT11:
    // In DHT22, humidity is (data[0]<<8 | data[1]) / 10, so data[0] is typically 0..3 for 0..100% RH.
    // In DHT11, data[0] is integer percentage (typically 20..90%).
    // Also in DHT11, fractional bytes data[1] and data[3] are 0..9. If data[1] > 9, it is definitely DHT22.
    if (dht22_h >= 0.0f && dht22_h <= 100.0f && dht22_t >= -40.0f && dht22_t <= 80.0f &&
        (data[0] <= 4 || data[1] > 9 || data[3] > 9)) {
        if (humidity_rh) *humidity_rh = dht22_h;
        if (temperature_c) *temperature_c = dht22_t;
        ESP_LOGD(TAG, "Decoded as DHT22: T=%.1f C, H=%.1f %%", dht22_t, dht22_h);
        return true;
    }

    if (dht11_h >= 0.0f && dht11_h <= 100.0f && dht11_t >= 0.0f && dht11_t <= 60.0f) {
        if (humidity_rh) *humidity_rh = dht11_h;
        if (temperature_c) *temperature_c = dht11_t;
        ESP_LOGD(TAG, "Decoded as DHT11: T=%.1f C, H=%.1f %%", dht11_t, dht11_h);
        return true;
    }

    if (dht22_h >= 0.0f && dht22_h <= 100.0f) {
        if (humidity_rh) *humidity_rh = dht22_h;
        if (temperature_c) *temperature_c = dht22_t;
        return true;
    }

    return false;
}

static esp_err_t read_dht_cached(float *out_temp, float *out_hum)
{
    int64_t now = esp_timer_get_time();
    // Cache is valid for 2 seconds (2,000,000 us)
    if (s_cache_valid && (now - s_last_read_time) >= 0 && (now - s_last_read_time) < 2000000) {
        if (out_temp) *out_temp = s_cached_temp;
        if (out_hum) *out_hum = s_cached_hum;
        return ESP_OK;
    }

    uint8_t data[5] = {0};
    esp_err_t err = dht_read_raw(APP_DHT_DATA_GPIO, data);
    if (err != ESP_OK) {
        // Retry once after 100ms
        vTaskDelay(pdMS_TO_TICKS(100));
        err = dht_read_raw(APP_DHT_DATA_GPIO, data);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Failed to read V182 sensor on GPIO %d (err=0x%x)", APP_DHT_DATA_GPIO, err);
            return err;
        }
    }

    float t = 0.0f;
    float h = 0.0f;
    if (!dht_decode(data, &t, &h)) {
        ESP_LOGW(TAG, "Failed to decode V182 sensor data");
        return ESP_ERR_INVALID_RESPONSE;
    }

    s_cached_temp = t;
    s_cached_hum = h;
    s_last_read_time = esp_timer_get_time();
    s_cache_valid = true;

    if (out_temp) *out_temp = t;
    if (out_hum) *out_hum = h;
    return ESP_OK;
}

esp_err_t sensor_init(void)
{
    s_cache_valid = false;
    s_last_read_time = 0;

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << APP_DHT_DATA_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&io_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure GPIO %d for V182 sensor: %s", APP_DHT_DATA_GPIO, esp_err_to_name(err));
        return err;
    }

    s_sensors_initialized = true;
    ESP_LOGI(TAG, "V182 (DHT) sensor initialized on GPIO %d (Wiring: '+' -> 3.3V, 'out' -> GPIO %d, '-' -> GND)",
             APP_DHT_DATA_GPIO, APP_DHT_DATA_GPIO);
    return ESP_OK;
}

bool sensor_is_initialized(void)
{
    return s_sensors_initialized;
}

esp_err_t sensor_read_temperature(float *temperature_c)
{
    return read_dht_cached(temperature_c, NULL);
}

esp_err_t sensor_read_humidity(float *humidity_rh)
{
    return read_dht_cached(NULL, humidity_rh);
}
