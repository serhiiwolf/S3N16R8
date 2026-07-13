#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "app_config.h"
#include "app_types.h"
#include "connectivity.h"
#include "sensors.h"

static const char *TAG = "interactive_mqtt";

void app_main(void)
{
    app_user_config_t user_cfg = {0};

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ota_confirm_running_image();

    ESP_ERROR_CHECK(app_console_init());
    ESP_ERROR_CHECK(sensor_init());

    if (app_is_config_reset_requested()) {
        ESP_LOGW(TAG, "Reset button pressed on GPIO%d: clearing saved config", 4);
        ESP_ERROR_CHECK(app_erase_user_config());
    }

    if (app_load_user_config(&user_cfg) != ESP_OK) {
        ESP_LOGI(TAG, "Saved configuration not found, switching to interactive setup");
        if (!app_read_user_config_interactive(&user_cfg)) {
            ESP_LOGE(TAG, "Configuration input failed");
            return;
        }

        ESP_ERROR_CHECK(app_save_user_config(&user_cfg));
        ESP_LOGI(TAG, "Configuration saved to NVS");
    } else {
        ESP_LOGI(TAG, "Loaded configuration from NVS");
    }

    if (wifi_init_sta(&user_cfg) != ESP_OK) {
        ESP_LOGE(TAG, "Wi-Fi connection failed. Reboot and try again.");
        return;
    }

    if (mqtt_start(&user_cfg) != ESP_OK) {
        ESP_LOGE(TAG, "MQTT start failed.");
        return;
    }

    ESP_LOGI(TAG, "Setup complete. Device is running.");
}
