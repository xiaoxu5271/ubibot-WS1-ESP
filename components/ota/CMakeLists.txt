idf_component_register(SRCS ./ota.c
                       INCLUDE_DIRS include                       # Edit following two lines to set component requirements (see docs)
                    #    REQUIRES )
                       PRIV_REQUIRES esp_http_client esp_https_ota app_update tasks user)

