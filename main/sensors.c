#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sensors.h"

#define APP_I2C_SDA_GPIO     8
#define APP_I2C_SCL_GPIO     9
#define APP_I2C_FREQ_HZ      100000

#define AHT20_ADDR           0x38

typedef struct {
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t aht20;
    bool initialized;
} sensor_context_t;

static const char *TAG = "interactive_mqtt";
static sensor_context_t s_sensors;

static esp_err_t aht20_init(i2c_master_dev_handle_t dev)
{
    const uint8_t init_cmd[] = { 0xBE, 0x08, 0x00 };
    esp_err_t err = i2c_master_transmit(dev, init_cmd, sizeof(init_cmd), 100);
    if (err != ESP_OK) {
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(20));
    return ESP_OK;
}

static esp_err_t aht20_read(i2c_master_dev_handle_t dev, float *temperature_c, float *humidity_rh)
{
    const uint8_t measure_cmd[] = { 0xAC, 0x33, 0x00 };
    uint8_t data[7] = { 0 };

    esp_err_t err = i2c_master_transmit(dev, measure_cmd, sizeof(measure_cmd), 100);
    if (err != ESP_OK) {
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(80));
    err = i2c_master_receive(dev, data, sizeof(data), 100);
    if (err != ESP_OK) {
        return err;
    }

    uint32_t raw_humidity = ((uint32_t)data[1] << 12) | ((uint32_t)data[2] << 4) | (data[3] >> 4);
    uint32_t raw_temperature = (((uint32_t)data[3] & 0x0F) << 16) | ((uint32_t)data[4] << 8) | data[5];

    if (humidity_rh != NULL) {
        *humidity_rh = (float)raw_humidity * 100.0f / 1048576.0f;
    }

    if (temperature_c != NULL) {
        *temperature_c = (float)raw_temperature * 200.0f / 1048576.0f - 50.0f;
    }

    return ESP_OK;
}

esp_err_t sensor_init(void)
{
    memset(&s_sensors, 0, sizeof(s_sensors));

    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = APP_I2C_SDA_GPIO,
        .scl_io_num = APP_I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = {
            .enable_internal_pullup = 1,
            .allow_pd = 0,
        },
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &s_sensors.bus));

    i2c_device_config_t aht20_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = AHT20_ADDR,
        .scl_speed_hz = APP_I2C_FREQ_HZ,
        .scl_wait_us = 0,
        .flags = {
            .disable_ack_check = 0,
        },
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(s_sensors.bus, &aht20_cfg, &s_sensors.aht20));
    ESP_ERROR_CHECK(i2c_master_probe(s_sensors.bus, AHT20_ADDR, 100));
    ESP_ERROR_CHECK(aht20_init(s_sensors.aht20));

    s_sensors.initialized = true;
    ESP_LOGI(TAG, "Sensor initialized: AHT20 at 0x38");
    return ESP_OK;
}

bool sensor_is_initialized(void)
{
    return s_sensors.initialized;
}

esp_err_t sensor_read_temperature(float *temperature_c)
{
    return aht20_read(s_sensors.aht20, temperature_c, NULL);
}

esp_err_t sensor_read_humidity(float *humidity_rh)
{
    return aht20_read(s_sensors.aht20, NULL, humidity_rh);
}
