#ifndef CONNECTIVITY_H
#define CONNECTIVITY_H

#include "esp_err.h"

#include "app_types.h"

esp_err_t wifi_init_sta(const app_user_config_t *cfg);
esp_err_t mqtt_start(const app_user_config_t *cfg);
void ota_confirm_running_image(void);

#endif
