file(GLOB SRC_LIST "*.c")
idf_component_register(SRCS ${SRC_LIST}
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES user esp_http_client json esp_adc_cal
)

