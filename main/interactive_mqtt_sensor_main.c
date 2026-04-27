#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/uart_vfs.h"
#include "esp_crt_bundle.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "mqtt_client.h"
#include "nvs.h"
#include "nvs_flash.h"

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
#define MAX_WIFI_RETRIES   10

#define APP_I2C_SDA_GPIO   8
#define APP_I2C_SCL_GPIO   9
#define APP_I2C_FREQ_HZ    100000

#define CONFIG_RESET_BUTTON_GPIO 4

#define AHT20_ADDR         0x38
#define BMP280_ADDR_PRIMARY 0x76
#define BMP280_ADDR_SECONDARY 0x77

#define MQTT_TOPIC_GET_TEMP     "getters/Temp"
#define MQTT_TOPIC_GET_HUMIDITY "getters/HUMIDITY"
#define MQTT_TOPIC_GET_PRESSURE "getters/PRESSURE"
#define MQTT_TOPIC_GET_ALTITUDE "getters/ALTITUDE"

#define MQTT_TOPIC_DATA_PREFIX   "data/"

#define NVS_NAMESPACE "appcfg"
#define NVS_KEY_CFG   "mqtt_cfg"

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

typedef struct {
    char wifi_ssid[33];
    char wifi_password[65];
    char broker_uri[128];
    char mqtt_client_id[64];
    char device_id[64];
} app_user_config_t;

static const char *TAG = "interactive_mqtt";
static EventGroupHandle_t s_wifi_event_group;
static int s_wifi_retry_num = 0;
static app_user_config_t s_user_cfg;
static sensor_context_t s_sensors;
static esp_mqtt_client_handle_t s_mqtt_client;

static bool is_config_reset_requested(void)
{
    gpio_config_t io_cfg = {
        .pin_bit_mask = (1ULL << CONFIG_RESET_BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_cfg));

    /* Button is active-low: hold it during boot to clear saved credentials. */
    vTaskDelay(pdMS_TO_TICKS(30));
    return gpio_get_level(CONFIG_RESET_BUTTON_GPIO) == 0;
}

static esp_err_t save_user_config_to_nvs(const app_user_config_t *cfg)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return err;
    }

    err = nvs_set_blob(handle, NVS_KEY_CFG, cfg, sizeof(*cfg));
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }

    nvs_close(handle);
    return err;
}

static esp_err_t load_user_config_from_nvs(app_user_config_t *cfg)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        return err;
    }

    size_t required_size = sizeof(*cfg);
    err = nvs_get_blob(handle, NVS_KEY_CFG, cfg, &required_size);
    nvs_close(handle);

    if (err != ESP_OK) {
        return err;
    }

    if (required_size != sizeof(*cfg)) {
        return ESP_ERR_NVS_INVALID_LENGTH;
    }

    if (strlen(cfg->wifi_ssid) == 0 || strlen(cfg->broker_uri) == 0 || strlen(cfg->mqtt_client_id) == 0) {
        return ESP_ERR_INVALID_STATE;
    }

    return ESP_OK;
}

static esp_err_t erase_user_config_from_nvs(void)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return err;
    }

    err = nvs_erase_key(handle, NVS_KEY_CFG);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        err = ESP_OK;
    }
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }

    nvs_close(handle);
    return err;
}

static esp_err_t init_console_io(void)
{
    uart_config_t uart_config = {
        .baud_rate = CONFIG_ESP_CONSOLE_UART_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err = uart_driver_install(CONFIG_ESP_CONSOLE_UART_NUM, 2048, 0, 0, NULL, 0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_ERROR_CHECK(uart_param_config(CONFIG_ESP_CONSOLE_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(CONFIG_ESP_CONSOLE_UART_NUM,
                                 UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE));

    uart_vfs_dev_use_driver(CONFIG_ESP_CONSOLE_UART_NUM);
    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);

    return ESP_OK;
}

static uint16_t read_u16_le(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static int16_t read_s16_le(const uint8_t *data)
{
    return (int16_t)read_u16_le(data);
}

static bool topic_matches(const char *topic, int topic_len, const char *expected)
{
    size_t expected_len = strlen(expected);
    return topic_len == (int)expected_len && memcmp(topic, expected, expected_len) == 0;
}

static void mqtt_publish_json_raw(const char *topic_suffix, const char *type, const char *data_json)
{
    if (s_mqtt_client == NULL) {
        ESP_LOGW(TAG, "MQTT client not ready, skip publish to %s", topic_suffix);
        return;
    }

    char topic[96];
    char payload[192];

    snprintf(topic, sizeof(topic), "%s%s", MQTT_TOPIC_DATA_PREFIX, topic_suffix);
    snprintf(payload, sizeof(payload), "{\"type\":\"%s\",\"data\":%s}", type, data_json);

    int msg_id = esp_mqtt_client_publish(s_mqtt_client, topic, payload, 0, 1, 0);
    ESP_LOGI(TAG, "Published %s (%d): %s", topic, msg_id, payload);
}

static void mqtt_publish_json_float(const char *topic_suffix, const char *type, float value, int precision)
{
    char data_json[32];
    snprintf(data_json, sizeof(data_json), "%.*f", precision, value);
    mqtt_publish_json_raw(topic_suffix, type, data_json);
}

static void mqtt_publish_json_error(const char *topic_suffix, const char *type)
{
    mqtt_publish_json_raw(topic_suffix, type, "\"read_error\"");
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

static esp_err_t sensor_init(void)
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

static void handle_getter_topic(const char *topic, int topic_len)
{
    float temperature_c = 0.0f;
    float humidity_rh = 0.0f;
    float pressure_hpa = 0.0f;
    float altitude_m = 0.0f;

    if (!s_sensors.initialized) {
        ESP_LOGE(TAG, "Sensors not initialized");
        return;
    }

    if (topic_matches(topic, topic_len, MQTT_TOPIC_GET_TEMP)) {
        if (aht20_read(s_sensors.aht20, &temperature_c, NULL) == ESP_OK) {
            mqtt_publish_json_float("Temp", "Temp", temperature_c, 2);
        } else {
            mqtt_publish_json_error("Temp", "Temp");
        }
    } else if (topic_matches(topic, topic_len, MQTT_TOPIC_GET_HUMIDITY)) {
        if (aht20_read(s_sensors.aht20, NULL, &humidity_rh) == ESP_OK) {
            mqtt_publish_json_float("HUMIDITY", "HUMIDITY", humidity_rh, 2);
        } else {
            mqtt_publish_json_error("HUMIDITY", "HUMIDITY");
        }
    } else if (topic_matches(topic, topic_len, MQTT_TOPIC_GET_PRESSURE)) {
        if (bmp280_read(s_sensors.bmp280, NULL, &pressure_hpa) == ESP_OK) {
            mqtt_publish_json_float("PRESSURE", "PRESSURE", pressure_hpa, 2);
        } else {
            mqtt_publish_json_error("PRESSURE", "PRESSURE");
        }
    } else if (topic_matches(topic, topic_len, MQTT_TOPIC_GET_ALTITUDE)) {
        if (bmp280_read(s_sensors.bmp280, NULL, &pressure_hpa) == ESP_OK && pressure_hpa > 0.0f) {
            altitude_m = 44330.0f * (1.0f - powf(pressure_hpa / 1013.25f, 0.1903f));
            mqtt_publish_json_float("ALTITUDE", "ALTITUDE", altitude_m, 2);
        } else {
            mqtt_publish_json_error("ALTITUDE", "ALTITUDE");
        }
    } else {
        ESP_LOGW(TAG, "Ignored topic: %.*s", topic_len, topic);
    }
}

static void trim_newline(char *text)
{
    size_t len = strlen(text);
    while (len > 0 && (text[len - 1] == '\n' || text[len - 1] == '\r')) {
        text[len - 1] = '\0';
        len--;
    }
}

static bool read_line_required(const char *prompt, char *out, size_t out_len)
{
    while (true) {
        printf("%s", prompt);
        fflush(stdout);

        if (fgets(out, out_len, stdin) == NULL) {
            ESP_LOGE(TAG, "Failed to read input from console");
            return false;
        }

        trim_newline(out);
        if (strlen(out) > 0) {
            return true;
        }

        printf("Value cannot be empty. Try again.\n");
    }
}

static bool read_user_config(app_user_config_t *cfg)
{
    printf("\n==== Device bootstrap setup ====\n");
    printf("Enter Wi-Fi and broker settings below.\n");
    printf("Broker URI examples: mqtt://192.168.1.20:1883 or wss://broker.example.com:443/mqtt\n\n");

    if (!read_line_required("Wi-Fi SSID: ", cfg->wifi_ssid, sizeof(cfg->wifi_ssid))) {
        return false;
    }

    if (!read_line_required("Wi-Fi Password: ", cfg->wifi_password, sizeof(cfg->wifi_password))) {
        return false;
    }

    if (!read_line_required("Broker URI (tcp/wss): ", cfg->broker_uri, sizeof(cfg->broker_uri))) {
        return false;
    }

    if (!read_line_required("MQTT Client ID: ", cfg->mqtt_client_id, sizeof(cfg->mqtt_client_id))) {
        return false;
    }

    if (!read_line_required("Device ID: ", cfg->device_id, sizeof(cfg->device_id))) {
        return false;
    }

    return true;
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_wifi_retry_num < MAX_WIFI_RETRIES) {
            esp_wifi_connect();
            s_wifi_retry_num++;
            ESP_LOGW(TAG, "Retrying Wi-Fi connection (%d/%d)", s_wifi_retry_num, MAX_WIFI_RETRIES);
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_wifi_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static esp_err_t wifi_init_sta(const app_user_config_t *cfg)
{
    s_wifi_event_group = xEventGroupCreate();
    if (s_wifi_event_group == NULL) {
        ESP_LOGE(TAG, "Failed to create Wi-Fi event group");
        return ESP_ERR_NO_MEM;
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t wifi_init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_init_cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_cfg = { 0 };
    strncpy((char *)wifi_cfg.sta.ssid, cfg->wifi_ssid, sizeof(wifi_cfg.sta.ssid) - 1);
    strncpy((char *)wifi_cfg.sta.password, cfg->wifi_password, sizeof(wifi_cfg.sta.password) - 1);
    wifi_cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_cfg.sta.pmf_cfg.capable = true;
    wifi_cfg.sta.pmf_cfg.required = false;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);

    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(s_wifi_event_group);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to AP: %s", cfg->wifi_ssid);
        return ESP_OK;
    }

    ESP_LOGE(TAG, "Failed to connect to AP: %s", cfg->wifi_ssid);
    return ESP_FAIL;
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
    esp_mqtt_client_handle_t client = event->client;

    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED: {
            ESP_LOGI(TAG, "MQTT connected");
            s_mqtt_client = client;

            esp_mqtt_client_subscribe(client, MQTT_TOPIC_GET_TEMP, 1);
            esp_mqtt_client_subscribe(client, MQTT_TOPIC_GET_HUMIDITY, 1);
            esp_mqtt_client_subscribe(client, MQTT_TOPIC_GET_PRESSURE, 1);
            esp_mqtt_client_subscribe(client, MQTT_TOPIC_GET_ALTITUDE, 1);

            char status_topic[128];
            char status_payload[160];
            snprintf(status_topic, sizeof(status_topic), "data/status");
            snprintf(status_payload, sizeof(status_payload), "{\"type\":\"status\",\"data\":{\"device_id\":\"%s\",\"state\":\"online\"}}", s_user_cfg.device_id);
            esp_mqtt_client_publish(client, status_topic, status_payload, 0, 1, 0);
            break;
        }
        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT subscribed, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT data on topic: %.*s", event->topic_len, event->topic);
            if (event->topic != NULL && event->topic_len > 0) {
                handle_getter_topic(event->topic, event->topic_len);
            }
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "MQTT disconnected");
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "MQTT event error");
            break;
        default:
            break;
    }
}

static esp_err_t mqtt_start(const app_user_config_t *cfg)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = cfg->broker_uri,
        .broker.verification.crt_bundle_attach = esp_crt_bundle_attach,
        .credentials.client_id = cfg->mqtt_client_id,
    };

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    if (client == NULL) {
        ESP_LOGE(TAG, "Failed to initialize MQTT client");
        return ESP_FAIL;
    }

    ESP_ERROR_CHECK(esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL));
    ESP_ERROR_CHECK(esp_mqtt_client_start(client));

    return ESP_OK;
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(init_console_io());
    ESP_ERROR_CHECK(sensor_init());

    if (is_config_reset_requested()) {
        ESP_LOGW(TAG, "Reset button pressed on GPIO%d: clearing saved config", CONFIG_RESET_BUTTON_GPIO);
        ESP_ERROR_CHECK(erase_user_config_from_nvs());
    }

    if (load_user_config_from_nvs(&s_user_cfg) != ESP_OK) {
        ESP_LOGI(TAG, "Saved configuration not found, switching to interactive setup");
        if (!read_user_config(&s_user_cfg)) {
            ESP_LOGE(TAG, "Configuration input failed");
            return;
        }

        ESP_ERROR_CHECK(save_user_config_to_nvs(&s_user_cfg));
        ESP_LOGI(TAG, "Configuration saved to NVS");
    } else {
        ESP_LOGI(TAG, "Loaded configuration from NVS");
    }

    if (wifi_init_sta(&s_user_cfg) != ESP_OK) {
        ESP_LOGE(TAG, "Wi-Fi connection failed. Reboot and try again.");
        return;
    }

    if (mqtt_start(&s_user_cfg) != ESP_OK) {
        ESP_LOGE(TAG, "MQTT start failed.");
        return;
    }

    ESP_LOGI(TAG, "Setup complete. Device is running.");
}
