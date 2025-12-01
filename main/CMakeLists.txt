idf_component_register(SRCS "util.c" "adc.c" "main.c"
                    PRIV_REQUIRES esp_adc nvs_flash driver hal esp_wifi esp_timer econsole
                    INCLUDE_DIRS ".")