idf_component_register(SRCS "lora_driver.c"
                       INCLUDE_DIRS "include"
                       REQUIRES common driver esp_driver_spi esp_driver_gpio)