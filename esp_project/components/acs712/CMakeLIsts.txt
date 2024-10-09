idf_component_register(SRCS "acs712.c" "acs712_filtered.c"
                        INCLUDE_DIRS "include"
                        REQUIRES esp_adc freertos)