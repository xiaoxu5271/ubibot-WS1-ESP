idf_component_register(SRCS ./Bluetooth.c
                        INCLUDE_DIRS "."                     # Edit following two lines to set component requirements (see docs)
                       REQUIRES json bt tasks user)

