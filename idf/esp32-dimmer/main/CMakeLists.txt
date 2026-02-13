idf_component_register(SRCS "esp32-dimmer.c"
                    PRIV_REQUIRES bt nvs_flash esp_driver_gpio esp_driver_gptimer
                    INCLUDE_DIRS ".")
