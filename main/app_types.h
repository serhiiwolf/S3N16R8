#ifndef APP_TYPES_H
#define APP_TYPES_H

typedef struct {
    char wifi_ssid[33];
    char wifi_password[65];
    char broker_uri[128];
    char mqtt_client_id[64];
    char device_id[64];
} app_user_config_t;

#endif
