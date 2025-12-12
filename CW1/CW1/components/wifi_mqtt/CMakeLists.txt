idf_component_register(SRCS "wifi_mqtt.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_wifi nvs_flash mqtt wpa_supplicant common)