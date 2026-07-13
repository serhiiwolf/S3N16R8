#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/uart_vfs.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"

#include "app_config.h"

#define CONFIG_RESET_BUTTON_GPIO 4
#define NVS_NAMESPACE "appcfg"
#define NVS_KEY_CFG   "mqtt_cfg"

static const char *TAG = "interactive_mqtt";

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

esp_err_t app_console_init(void)
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

bool app_is_config_reset_requested(void)
{
    gpio_config_t io_cfg = {
        .pin_bit_mask = (1ULL << CONFIG_RESET_BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_cfg));

    vTaskDelay(pdMS_TO_TICKS(30));
    return gpio_get_level(CONFIG_RESET_BUTTON_GPIO) == 0;
}

esp_err_t app_save_user_config(const app_user_config_t *cfg)
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

esp_err_t app_load_user_config(app_user_config_t *cfg)
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

esp_err_t app_erase_user_config(void)
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

bool app_read_user_config_interactive(app_user_config_t *cfg)
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
