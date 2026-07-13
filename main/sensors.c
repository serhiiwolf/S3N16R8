#include <math.h>
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
#define BMP280_ADDR_PRIMARY  0x76
#define BMP280_ADDR_SECONDARY 0x77

typedef struct {
    uint16_t dig_t1;
    int16_t dig_t2;
    int16_t dig_t3;
    uint16_t dig_p1;
    int16_t dig_p2;
    int16_t dig_p3;
    int16_t dig_p4;
    int16_t dig_p5;
    int16_t dig_p6;
    int16_t dig_p7;
    int16_t dig_p8;
    int16_t dig_p9;
    int32_t t_fine;
} bmp280_calib_t;

typedef struct {
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t aht20;
    i2c_master_dev_handle_t bmp280;
    uint8_t bmp280_address;
    bmp280_calib_t bmp280_calib;
    bool initialized;
} sensor_context_t;

static const char *TAG = "interactive_mqtt";
static sensor_context_t s_sensors;

static uint16_t read_u16_le(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static int16_t read_s16_le(const uint8_t *data)
{
    return (int16_t)read_u16_le(data);
}

static esp_err_t i2c_write_reg(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t value)
{
    uint8_t buffer[2] = { reg, value };
    return i2c_master_transmit(dev, buffer, sizeof(buffer), 100);
}

static esp_err_t i2c_read_reg(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(dev, &reg, 1, data, len, 100);
}

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

static esp_err_t bmp280_read_calibration(i2c_master_dev_handle_t dev, bmp280_calib_t *calib)
{
    uint8_t data[24] = { 0 };
    esp_err_t err = i2c_read_reg(dev, 0x88, data, sizeof(data));
    if (err != ESP_OK) {
        return err;
    }

    calib->dig_t1 = read_u16_le(&data[0]);
    calib->dig_t2 = read_s16_le(&data[2]);
    calib->dig_t3 = read_s16_le(&data[4]);
    calib->dig_p1 = read_u16_le(&data[6]);
    calib->dig_p2 = read_s16_le(&data[8]);
    calib->dig_p3 = read_s16_le(&data[10]);
    calib->dig_p4 = read_s16_le(&data[12]);
    calib->dig_p5 = read_s16_le(&data[14]);
    calib->dig_p6 = read_s16_le(&data[16]);
    calib->dig_p7 = read_s16_le(&data[18]);
    calib->dig_p8 = read_s16_le(&data[20]);
    calib->dig_p9 = read_s16_le(&data[22]);

    return ESP_OK;
}

static float bmp280_compensate_temperature_c(bmp280_calib_t *calib, int32_t adc_temperature)
{
    int32_t var1 = ((((adc_temperature >> 3) - ((int32_t)calib->dig_t1 << 1))) * ((int32_t)calib->dig_t2)) >> 11;
    int32_t var2 = (((((adc_temperature >> 4) - ((int32_t)calib->dig_t1)) * ((adc_temperature >> 4) - ((int32_t)calib->dig_t1))) >> 12) * ((int32_t)calib->dig_t3)) >> 14;
    calib->t_fine = var1 + var2;

    int32_t temperature = (calib->t_fine * 5 + 128) >> 8;
    return (float)temperature / 100.0f;
}

static float bmp280_compensate_pressure_pa(bmp280_calib_t *calib, int32_t adc_pressure)
{
    int64_t var1 = ((int64_t)calib->t_fine) - 128000;
    int64_t var2 = var1 * var1 * (int64_t)calib->dig_p6;
    var2 = var2 + ((var1 * (int64_t)calib->dig_p5) << 17);
    var2 = var2 + (((int64_t)calib->dig_p4) << 35);
    var1 = ((var1 * var1 * (int64_t)calib->dig_p3) >> 8) + ((var1 * (int64_t)calib->dig_p2) << 12);
    var1 = (((((int64_t)1) << 47) + var1) * ((int64_t)calib->dig_p1)) >> 33;

    if (var1 == 0) {
        return 0.0f;
    }

    int64_t pressure = 1048576 - adc_pressure;
    pressure = (((pressure << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)calib->dig_p9) * (pressure >> 13) * (pressure >> 13)) >> 25;
    var2 = (((int64_t)calib->dig_p8) * pressure) >> 19;
    pressure = ((pressure + var1 + var2) >> 8) + (((int64_t)calib->dig_p7) << 4);

    return (float)pressure / 256.0f;
}

static esp_err_t bmp280_init(i2c_master_dev_handle_t dev)
{
    uint8_t chip_id = 0;
    esp_err_t err = i2c_read_reg(dev, 0xD0, &chip_id, 1);
    if (err != ESP_OK) {
        return err;
    }

    if (chip_id != 0x58) {
        ESP_LOGW(TAG, "Unexpected BMP280 chip id: 0x%02x", chip_id);
    }

    err = bmp280_read_calibration(dev, &s_sensors.bmp280_calib);
    if (err != ESP_OK) {
        return err;
    }

    err = i2c_write_reg(dev, 0xF5, 0x00);
    if (err != ESP_OK) {
        return err;
    }

    return ESP_OK;
}

static esp_err_t bmp280_read(i2c_master_dev_handle_t dev, float *temperature_c, float *pressure_hpa)
{
    esp_err_t err = i2c_write_reg(dev, 0xF4, 0x25);
    if (err != ESP_OK) {
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(10));

    uint8_t data[6] = { 0 };
    err = i2c_read_reg(dev, 0xF7, data, sizeof(data));
    if (err != ESP_OK) {
        return err;
    }

    int32_t adc_pressure = ((int32_t)data[0] << 12) | ((int32_t)data[1] << 4) | (data[2] >> 4);
    int32_t adc_temperature = ((int32_t)data[3] << 12) | ((int32_t)data[4] << 4) | (data[5] >> 4);

    if (temperature_c != NULL) {
        *temperature_c = bmp280_compensate_temperature_c(&s_sensors.bmp280_calib, adc_temperature);
    } else {
        (void)bmp280_compensate_temperature_c(&s_sensors.bmp280_calib, adc_temperature);
    }

    if (pressure_hpa != NULL) {
        float pressure_pa = bmp280_compensate_pressure_pa(&s_sensors.bmp280_calib, adc_pressure);
        *pressure_hpa = pressure_pa / 100.0f;
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

    uint8_t bmp280_address = BMP280_ADDR_PRIMARY;
    if (i2c_master_probe(s_sensors.bus, bmp280_address, 100) != ESP_OK) {
        bmp280_address = BMP280_ADDR_SECONDARY;
    }

    ESP_ERROR_CHECK(i2c_master_probe(s_sensors.bus, bmp280_address, 100));
    s_sensors.bmp280_address = bmp280_address;

    i2c_device_config_t bmp280_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = bmp280_address,
        .scl_speed_hz = APP_I2C_FREQ_HZ,
        .scl_wait_us = 0,
        .flags = {
            .disable_ack_check = 0,
        },
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(s_sensors.bus, &bmp280_cfg, &s_sensors.bmp280));
    ESP_ERROR_CHECK(bmp280_init(s_sensors.bmp280));

    s_sensors.initialized = true;
    ESP_LOGI(TAG, "Sensors initialized: AHT20 at 0x38, BMP280 at 0x%02x", bmp280_address);
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

esp_err_t sensor_read_pressure(float *pressure_hpa)
{
    return bmp280_read(s_sensors.bmp280, NULL, pressure_hpa);
}

esp_err_t sensor_read_altitude(float *altitude_m)
{
    float pressure_hpa = 0.0f;
    esp_err_t err = sensor_read_pressure(&pressure_hpa);
    if (err != ESP_OK) {
        return err;
    }

    if (pressure_hpa <= 0.0f) {
        return ESP_FAIL;
    }

    if (altitude_m != NULL) {
        *altitude_m = 44330.0f * (1.0f - powf(pressure_hpa / 1013.25f, 0.1903f));
    }

    return ESP_OK;
}
