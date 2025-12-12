#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "mqtt_client.h"
#include "esp_sntp.h"

#include "wifi_mqtt.h"

static const char *TAG = "WIFI_MQTT";

// Global MQTT client handle
esp_mqtt_client_handle_t client = NULL;

// ============================================================
// MQTT Statistics - Track everything!
// ============================================================
static uint32_t mqtt_publish_count = 0;      // Total publish attempts
static uint32_t mqtt_publish_success = 0;    // Successful publishes
static uint32_t mqtt_publish_fail = 0;       // Failed publishes
static uint32_t mqtt_connect_count = 0;      // Connection count
static uint32_t mqtt_disconnect_count = 0;   // Disconnection count
static uint32_t mqtt_stats_minute_counter = 0; // For 1-minute CSV dump

// NTP sync semaphore
static SemaphoreHandle_t s_sync_sem = NULL;

// Forward declaration
void mqtt_app_start(void);

/**
 * NTP time sync callback
 */
static void time_sync_notification_cb(struct timeval *tv) {
    (void)tv;
    ESP_LOGI(TAG, "SNTP sync event received - Time is ACCURATE now!");
    if (s_sync_sem) {
        xSemaphoreGive(s_sync_sem);
    }
}

/**
 * Initialize SNTP for time synchronization
 */
static void init_sntp(void) {
    ESP_LOGI(TAG, "Initializing SNTP");
    
    // Step 1: Create semaphore for sync waiting
    s_sync_sem = xSemaphoreCreateBinary();
    if (s_sync_sem == NULL) {
        ESP_LOGE(TAG, "Could not create SNTP semaphore");
        return;
    }
    
    // Step 2: Stop old SNTP, set mode and callback
    esp_sntp_stop();
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_set_time_sync_notification_cb(time_sync_notification_cb);
    
    // Configure NTP servers
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_setservername(1, "europe.pool.ntp.org");
    esp_sntp_setservername(2, "time.google.com");
    
    // Step 4: Initialize
    esp_sntp_init();
    
    // Set timezone to UTC
    setenv("TZ", "GMT0", 1);
    tzset();
    
    // Wait for sync (max 30 seconds)
    ESP_LOGI(TAG, "Waiting for NTP time sync...");
    if (xSemaphoreTake(s_sync_sem, pdMS_TO_TICKS(30000)) == pdTRUE) {
        time_t now;
        struct tm timeinfo;
        time(&now);
        localtime_r(&now, &timeinfo);
        ESP_LOGI(TAG, "NTP Time synced: %04d-%02d-%02d %02d:%02d:%02d UTC",
                 timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
                 timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
    } else {
        ESP_LOGW(TAG, "NTP sync timeout (30s) - Using boot time instead!");
    }
    
    // Step 7: Cleanup
    sntp_set_time_sync_notification_cb(NULL);
    vSemaphoreDelete(s_sync_sem);
    s_sync_sem = NULL;
}

/**
 * WiFi event handler
 */
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                          int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t* event = (wifi_event_sta_disconnected_t*) event_data;
        ESP_LOGE(TAG, "WiFi Disconnected. Reason: %d", event->reason);
        esp_wifi_connect();
        ESP_LOGI(TAG, "Retrying WiFi connection");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP:" IPSTR, IP2STR(&event->ip_info.ip));

        // WiFi connected! Now sync time, then start MQTT
        ESP_LOGI(TAG, "WiFi Connected! Syncing NTP time...");
        init_sntp();
        
        ESP_LOGI(TAG, "Starting MQTT client");
        mqtt_app_start();
    }
}

/**
 * MQTT event handler - Tracks connection status
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = event_data;
    if (event->event_id == MQTT_EVENT_CONNECTED) {
        mqtt_connect_count++;
        ESP_LOGI(TAG, "MQTT CONNECTED (Connect #%lu)", mqtt_connect_count);
    } else if (event->event_id == MQTT_EVENT_DISCONNECTED) {
        mqtt_disconnect_count++;
        ESP_LOGI(TAG, "MQTT DISCONNECTED (Disconnect #%lu)", mqtt_disconnect_count);
    } else if (event->event_id == MQTT_EVENT_PUBLISHED) {
        mqtt_publish_success++;
    }
}

/**
 * Initialize WiFi Station mode
 */
void wifi_init_sta(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config = {0};

#ifdef USE_EDUROAM
    // Eduroam mode
    #include "esp_eap_client.h" 

    ESP_LOGI(TAG, "Configuring for eduroam");
    
    strcpy((char *)wifi_config.sta.ssid, EDUROAM_SSID);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_ENTERPRISE;
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

    ESP_ERROR_CHECK(esp_eap_client_set_identity((uint8_t *)EDUROAM_IDENTITY, strlen(EDUROAM_IDENTITY)));
    ESP_ERROR_CHECK(esp_eap_client_set_username((uint8_t *)EDUROAM_USERNAME, strlen(EDUROAM_USERNAME)));
    ESP_ERROR_CHECK(esp_eap_client_set_password((uint8_t *)EDUROAM_PASSWORD, strlen(EDUROAM_PASSWORD)));
    
    ESP_ERROR_CHECK(esp_wifi_sta_enterprise_enable());

#else
    // Home WiFi mode
    ESP_LOGI(TAG, "Configuring for Home WiFi: %s", HOME_WIFI_SSID);
    
    strcpy((char *)wifi_config.sta.ssid, HOME_WIFI_SSID);
    strcpy((char *)wifi_config.sta.password, HOME_WIFI_PASS);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
#endif

    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "WiFi initialization complete");
}

/**
 * Start MQTT client
 */
void mqtt_app_start(void) {
    // Prevent double initialization
    if (client != NULL) {
        ESP_LOGW(TAG, "MQTT client already running");
        return;
    }

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_BROKER_URI,
    };
    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
    esp_mqtt_client_start(client);
}

/**
 * Publish telemetry to MQTT
 */
void mqtt_publish_telemetry(char *json_payload) {
    mqtt_publish_count++;
    
    if (client != NULL) {
        // QoS 1, no retain
        int msg_id = esp_mqtt_client_publish(client, MQTT_TOPIC, json_payload, 0, 1, 0);
        if (msg_id < 0) {
            mqtt_publish_fail++;
            ESP_LOGW(TAG, "MQTT publish failed!");
        }
    } else {
        mqtt_publish_fail++;
    }
    
    // 1-minute MQTT stats CSV dump (every ~120 calls at 2Hz telemetry rate)
    mqtt_stats_minute_counter++;
    if (mqtt_stats_minute_counter >= 120) {
        mqtt_stats_minute_counter = 0;
        float success_rate = (mqtt_publish_count > 0) ? 
            (100.0f * mqtt_publish_success / mqtt_publish_count) : 0;
        printf("CSV_MQTT,%lu,%lu,%lu,%.1f,%lu,%lu\n",
               mqtt_publish_count, mqtt_publish_success, mqtt_publish_fail,
               success_rate, mqtt_connect_count, mqtt_disconnect_count);
        ESP_LOGI(TAG, "MQTT Stats: Pub=%lu Success=%lu Fail=%lu Rate=%.1f%%",
                 mqtt_publish_count, mqtt_publish_success, mqtt_publish_fail, success_rate);
    }
}

/**
 * Get current Unix timestamp with milliseconds
 */
void get_current_unix_time(uint32_t *ts_s, uint16_t *ts_ms) {
    struct timeval tv;
    if (gettimeofday(&tv, NULL) == 0) {
        *ts_s  = (uint32_t)tv.tv_sec;
        *ts_ms = (uint16_t)(tv.tv_usec / 1000);
    } else {
        ESP_LOGE(TAG, "gettimeofday failed");
        *ts_s = 0;
        *ts_ms = 0;
    }
}