#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <stdbool.h>

#include "esp_err.h"

#include "app_types.h"

esp_err_t app_console_init(void);
bool app_is_config_reset_requested(void);
esp_err_t app_save_user_config(const app_user_config_t *cfg);
esp_err_t app_load_user_config(app_user_config_t *cfg);
esp_err_t app_erase_user_config(void);
bool app_read_user_config_interactive(app_user_config_t *cfg);

#endif
