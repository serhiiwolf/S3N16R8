#ifndef SENSORS_H
#define SENSORS_H

#include <stdbool.h>

#include "esp_err.h"

esp_err_t sensor_init(void);
bool sensor_is_initialized(void);
esp_err_t sensor_read_temperature(float *temperature_c);
esp_err_t sensor_read_humidity(float *humidity_rh);
esp_err_t sensor_read_pressure(float *pressure_hpa);
esp_err_t sensor_read_altitude(float *altitude_m);

#endif
