idf_component_register(SRCS "interactive_mqtt_sensor_main.c"
                            "app_config.c"
                            "sensors.c"
                            "connectivity.c"
                       PRIV_REQUIRES esp_wifi mqtt esp_netif esp_event nvs_flash esp-tls esp_https_ota app_update esp_driver_uart esp_driver_i2c esp_driver_gpio
                       INCLUDE_DIRS "")
