idf_component_register(SRCS "util.c" "adc.c" "main.c"
                    PRIV_REQUIRES esp_adc esp_console nvs_flash driver hal wifi esp_timer gpio
                    INCLUDE_DIRS ".")
