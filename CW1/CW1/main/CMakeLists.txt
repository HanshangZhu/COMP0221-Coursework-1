idf_component_register(SRCS "CW1.c"
                       INCLUDE_DIRS "."
                       REQUIRES common lora_driver wifi_mqtt security nvs_flash esp_wifi mqtt)