file(GLOB SRC_LIST "*.c")
idf_component_register(SRCS ${SRC_LIST}
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES tasks nvs_flash Bluetooth
)

