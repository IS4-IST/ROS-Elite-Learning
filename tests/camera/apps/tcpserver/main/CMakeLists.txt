
idf_component_register(SRCS tcpserver.cpp
                        PRIV_INCLUDE_DIRS .
                        PRIV_REQUIRES esp_wifi nvs_flash)


target_compile_options(${COMPONENT_LIB} PRIVATE -std=gnu++11)