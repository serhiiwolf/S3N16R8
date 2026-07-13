#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "esp_crt_bundle.h"
#include "esp_event.h"
#include "esp_https_ota.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "mqtt_client.h"

#include "connectivity.h"
#include "sensors.h"

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
#define MAX_WIFI_RETRIES   10
#define MQTT_TOPIC_METRICS_PREFIX "Metrics"
#define MQTT_TOPIC_OTA_UPDATE_PREFIX "OTA/update"
#define MQTT_TOPIC_OTA_STATUS_PREFIX "OTA/status"

#define OTA_URL_MAX_LEN 256
#define OTA_SHA256_MAX_LEN 65

typedef struct {
    char url[OTA_URL_MAX_LEN];
    char sha256[OTA_SHA256_MAX_LEN];
} ota_job_t;

static const char *TAG = "interactive_mqtt";

static EventGroupHandle_t s_wifi_event_group;
static int s_wifi_retry_num = 0;
static bool s_wifi_connected_once = false;
static bool s_ota_in_progress = false;
static app_user_config_t s_user_cfg;
static esp_mqtt_client_handle_t s_mqtt_client;

static void mqtt_publish_json_to_topic(const char *topic, int topic_len, const char *type, const char *data_json)
{
    if (s_mqtt_client == NULL) {
        ESP_LOGW(TAG, "MQTT client not ready, skip publish to %.*s", topic_len, topic);
        return;
    }

    char topic_buf[128] = {0};
    if (topic_len >= (int)sizeof(topic_buf)) {
        ESP_LOGW(TAG, "Topic too long, skip publish");
        return;
    }
    memcpy(topic_buf, topic, topic_len);
    topic_buf[topic_len] = '\0';

    char payload[192];
    snprintf(payload, sizeof(payload), "{\"type\":\"%s\",\"data\":%s}", type, data_json);

    int msg_id = esp_mqtt_client_publish(s_mqtt_client, topic_buf, payload, 0, 1, 0);
    ESP_LOGI(TAG, "Published %s (%d): %s", topic_buf, msg_id, payload);
}

static void build_ota_topic(char *out, size_t out_len, const char *prefix)
{
    snprintf(out, out_len, "%s/%s", prefix, s_user_cfg.device_id);
}

static bool topic_matches_device(const char *topic, int topic_len, const char *prefix)
{
    char expected[128];
    build_ota_topic(expected, sizeof(expected), prefix);
    size_t expected_len = strlen(expected);

    return topic_len == (int)expected_len && strncmp(topic, expected, expected_len) == 0;
}

static void publish_ota_status(const char *status, const char *detail)
{
    char topic[128];
    char payload[320];

    build_ota_topic(topic, sizeof(topic), MQTT_TOPIC_OTA_STATUS_PREFIX);
    snprintf(payload,
             sizeof(payload),
             "{\"type\":\"ota\",\"data\":{\"status\":\"%s\",\"detail\":\"%s\"}}",
             status,
             detail == NULL ? "" : detail);

    if (s_mqtt_client != NULL) {
        esp_mqtt_client_publish(s_mqtt_client, topic, payload, 0, 1, 0);
    }
}

static void trim_whitespace(char *text)
{
    size_t len = strlen(text);
    while (len > 0 && (text[len - 1] == ' ' || text[len - 1] == '\t' || text[len - 1] == '\r' || text[len - 1] == '\n')) {
        text[len - 1] = '\0';
        len--;
    }

    size_t start = 0;
    while (text[start] == ' ' || text[start] == '\t' || text[start] == '\r' || text[start] == '\n') {
        start++;
    }

    if (start > 0) {
        memmove(text, &text[start], strlen(&text[start]) + 1);
    }
}

static bool extract_json_field(const char *json, const char *field_name, char *out, size_t out_len)
{
    char key[32];
    snprintf(key, sizeof(key), "\"%s\"", field_name);

    const char *key_pos = strstr(json, key);
    if (key_pos == NULL) {
        return false;
    }

    const char *colon = strchr(key_pos, ':');
    if (colon == NULL) {
        return false;
    }

    const char *first_quote = strchr(colon, '"');
    if (first_quote == NULL) {
        return false;
    }
    first_quote++;

    const char *second_quote = strchr(first_quote, '"');
    if (second_quote == NULL) {
        return false;
    }

    size_t len = (size_t)(second_quote - first_quote);
    if (len == 0 || len >= out_len) {
        return false;
    }

    memcpy(out, first_quote, len);
    out[len] = '\0';
    return true;
}

static bool parse_ota_payload(const char *payload, int payload_len, ota_job_t *job)
{
    if (payload == NULL || payload_len <= 0 || payload_len >= OTA_URL_MAX_LEN) {
        return false;
    }

    char buffer[OTA_URL_MAX_LEN];
    memcpy(buffer, payload, payload_len);
    buffer[payload_len] = '\0';
    trim_whitespace(buffer);

    memset(job, 0, sizeof(*job));

    if (buffer[0] == '{') {
        if (!extract_json_field(buffer, "url", job->url, sizeof(job->url))) {
            return false;
        }
        (void)extract_json_field(buffer, "sha256", job->sha256, sizeof(job->sha256));
    } else {
        strncpy(job->url, buffer, sizeof(job->url) - 1);
    }

    if (strncmp(job->url, "https://", 8) != 0) {
        return false;
    }

    return true;
}

static bool parse_metrics_topic(const char *topic, int topic_len, char *metric_out, size_t metric_out_len)
{
    const char *prefix = MQTT_TOPIC_METRICS_PREFIX "/";
    size_t prefix_len = strlen(prefix);
    if (topic_len <= (int)prefix_len + 2) {
        return false;
    }

    if (topic_len < (int)prefix_len) {
        return false;
    }

    if (strncmp(topic, prefix, prefix_len) != 0) {
        return false;
    }

    int i = (int)prefix_len;
    int slash_pos = -1;
    for (; i < topic_len; i++) {
        if (topic[i] == '/') {
            slash_pos = i;
            break;
        }
    }
    if (slash_pos <= 0) {
        return false;
    }

    int metric_len = slash_pos - (int)prefix_len;
    int device_len = topic_len - (slash_pos + 1);
    if (metric_len <= 0 || device_len <= 0) {
        return false;
    }

    if ((size_t)device_len != strlen(s_user_cfg.device_id)) {
        return false;
    }
    if (strncmp(&topic[slash_pos + 1], s_user_cfg.device_id, device_len) != 0) {
        return false;
    }

    if ((size_t)metric_len >= metric_out_len) {
        return false;
    }
    memcpy(metric_out, &topic[prefix_len], metric_len);
    metric_out[metric_len] = '\0';
    return true;
}

static bool is_metrics_request_payload(const char *payload, int payload_len)
{
    if (payload == NULL || payload_len == 0) {
        return true;
    }

    int i = 0;
    while (i < payload_len && (payload[i] == ' ' || payload[i] == '\t' || payload[i] == '\r' || payload[i] == '\n')) {
        i++;
    }

    if (i >= payload_len) {
        return true;
    }

    if (payload[i] == '{' || payload[i] == '[') {
        return false;
    }

    const int remaining = payload_len - i;
    if (remaining == 3 &&
        (payload[i] == 'g' || payload[i] == 'G') &&
        (payload[i + 1] == 'e' || payload[i + 1] == 'E') &&
        (payload[i + 2] == 't' || payload[i + 2] == 'T')) {
        return true;
    }

    if (remaining == 1 && payload[i] == '1') {
        return true;
    }

    return false;
}

static void handle_metrics_topic(const char *topic, int topic_len, const char *payload, int payload_len)
{
    if (!sensor_is_initialized()) {
        ESP_LOGE(TAG, "Sensors not initialized");
        return;
    }

    if (!is_metrics_request_payload(payload, payload_len)) {
        ESP_LOGD(TAG, "Ignored non-request payload on topic: %.*s", topic_len, topic);
        return;
    }

    char metric[32];
    if (!parse_metrics_topic(topic, topic_len, metric, sizeof(metric))) {
        ESP_LOGW(TAG, "Ignored topic: %.*s", topic_len, topic);
        return;
    }

    if (strcmp(metric, "status") == 0) {
        return;
    }

    if (strcmp(metric, "Temp") == 0) {
        float temperature_c = 0.0f;
        if (sensor_read_temperature(&temperature_c) == ESP_OK) {
            char data_json[32];
            snprintf(data_json, sizeof(data_json), "%.*f", 2, temperature_c);
            mqtt_publish_json_to_topic(topic, topic_len, "Temp", data_json);
        } else {
            mqtt_publish_json_to_topic(topic, topic_len, "Temp", "\"read_error\"");
        }
    } else if (strcmp(metric, "HUMIDITY") == 0) {
        float humidity_rh = 0.0f;
        if (sensor_read_humidity(&humidity_rh) == ESP_OK) {
            char data_json[32];
            snprintf(data_json, sizeof(data_json), "%.*f", 2, humidity_rh);
            mqtt_publish_json_to_topic(topic, topic_len, "HUMIDITY", data_json);
        } else {
            mqtt_publish_json_to_topic(topic, topic_len, "HUMIDITY", "\"read_error\"");
        }
    } else if (strcmp(metric, "PRESSURE") == 0) {
        float pressure_hpa = 0.0f;
        if (sensor_read_pressure(&pressure_hpa) == ESP_OK) {
            char data_json[32];
            snprintf(data_json, sizeof(data_json), "%.*f", 2, pressure_hpa);
            mqtt_publish_json_to_topic(topic, topic_len, "PRESSURE", data_json);
        } else {
            mqtt_publish_json_to_topic(topic, topic_len, "PRESSURE", "\"read_error\"");
        }
    } else if (strcmp(metric, "ALTITUDE") == 0) {
        float altitude_m = 0.0f;
        if (sensor_read_altitude(&altitude_m) == ESP_OK) {
            char data_json[32];
            snprintf(data_json, sizeof(data_json), "%.*f", 2, altitude_m);
            mqtt_publish_json_to_topic(topic, topic_len, "ALTITUDE", data_json);
        } else {
            mqtt_publish_json_to_topic(topic, topic_len, "ALTITUDE", "\"read_error\"");
        }
    } else {
        ESP_LOGW(TAG, "Unknown metric requested: %s", metric);
    }
}

static void ota_update_task(void *arg)
{
    ota_job_t *job = (ota_job_t *)arg;

    ESP_LOGI(TAG, "Starting OTA from URL: %s", job->url);
    publish_ota_status("downloading", "started");

    esp_http_client_config_t http_cfg = {
        .url = job->url,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms = 15000,
        .keep_alive_enable = true,
    };

    esp_https_ota_config_t ota_cfg = {
        .http_config = &http_cfg,
    };

    esp_err_t err = esp_https_ota(&ota_cfg);
    if (err == ESP_OK) {
        publish_ota_status("applied", "rebooting");
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_restart();
    } else {
        ESP_LOGE(TAG, "OTA failed: %s", esp_err_to_name(err));
        publish_ota_status("failed", esp_err_to_name(err));
        s_ota_in_progress = false;
    }

    free(job);
    vTaskDelete(NULL);
}

static void handle_ota_topic(const char *payload, int payload_len)
{
    if (s_ota_in_progress) {
        publish_ota_status("busy", "update_in_progress");
        return;
    }

    ota_job_t *job = (ota_job_t *)calloc(1, sizeof(ota_job_t));
    if (job == NULL) {
        publish_ota_status("failed", "no_memory");
        return;
    }

    if (!parse_ota_payload(payload, payload_len, job)) {
        publish_ota_status("failed", "bad_payload");
        free(job);
        return;
    }

    s_ota_in_progress = true;
    publish_ota_status("accepted", "queued");

    BaseType_t task_ok = xTaskCreate(ota_update_task, "ota_update", 8192, job, 5, NULL);
    if (task_ok != pdPASS) {
        s_ota_in_progress = false;
        publish_ota_status("failed", "task_create_failed");
        free(job);
    }
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        s_mqtt_client = NULL;

        if (s_wifi_connected_once) {
            esp_wifi_connect();
            ESP_LOGW(TAG, "Wi-Fi disconnected, retrying...");
        } else {
            if (s_wifi_retry_num < MAX_WIFI_RETRIES) {
                esp_wifi_connect();
                s_wifi_retry_num++;
                ESP_LOGW(TAG, "Retrying Wi-Fi connection (%d/%d)", s_wifi_retry_num, MAX_WIFI_RETRIES);
            } else if (s_wifi_event_group != NULL) {
                xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            }
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_wifi_retry_num = 0;
        s_wifi_connected_once = true;
        if (s_wifi_event_group != NULL) {
            xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        }
    }
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
    esp_mqtt_client_handle_t client = event->client;

    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED: {
            ESP_LOGI(TAG, "MQTT connected");
            s_mqtt_client = client;

            char final_sub[128];
            snprintf(final_sub, sizeof(final_sub), "%s/+/%s", MQTT_TOPIC_METRICS_PREFIX, s_user_cfg.device_id);
            esp_mqtt_client_subscribe(client, final_sub, 1);

            char ota_sub[128];
            build_ota_topic(ota_sub, sizeof(ota_sub), MQTT_TOPIC_OTA_UPDATE_PREFIX);
            esp_mqtt_client_subscribe(client, ota_sub, 1);

            char status_topic[128];
            char status_payload[160];
            snprintf(status_topic, sizeof(status_topic), "%s/status/%s", MQTT_TOPIC_METRICS_PREFIX, s_user_cfg.device_id);
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
                if (topic_matches_device(event->topic, event->topic_len, MQTT_TOPIC_OTA_UPDATE_PREFIX)) {
                    handle_ota_topic(event->data, event->data_len);
                } else {
                    handle_metrics_topic(event->topic, event->topic_len, event->data, event->data_len);
                }
            }
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "MQTT disconnected");
            s_mqtt_client = NULL;
            esp_mqtt_client_reconnect(client);
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "MQTT event error");
            break;
        default:
            break;
    }
}

esp_err_t wifi_init_sta(const app_user_config_t *cfg)
{
    if (s_wifi_event_group == NULL) {
        s_wifi_event_group = xEventGroupCreate();
        if (s_wifi_event_group == NULL) {
            ESP_LOGE(TAG, "Failed to create Wi-Fi event group");
            return ESP_ERR_NO_MEM;
        }
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

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to AP: %s", cfg->wifi_ssid);
        return ESP_OK;
    }

    ESP_LOGE(TAG, "Failed to connect to AP: %s", cfg->wifi_ssid);
    return ESP_FAIL;
}

esp_err_t mqtt_start(const app_user_config_t *cfg)
{
    s_user_cfg = *cfg;

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = cfg->broker_uri,
        .broker.verification.crt_bundle_attach = esp_crt_bundle_attach,
        .credentials.client_id = cfg->mqtt_client_id,
        .network.disable_auto_reconnect = false,
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

void ota_confirm_running_image(void)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    if (running == NULL) {
        return;
    }

    esp_ota_img_states_t ota_state;
    if (esp_ota_get_state_partition(running, &ota_state) != ESP_OK) {
        return;
    }

    if (ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
        esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Marked OTA image valid");
        } else {
            ESP_LOGW(TAG, "Failed to mark OTA image valid: %s", esp_err_to_name(err));
        }
    }
}
