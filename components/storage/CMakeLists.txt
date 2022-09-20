idf_component_register( SRCS "storage.c"
                        INCLUDE_DIRS "include"
                        REQUIRES nvs_flash hydrate_common
                      )

target_compile_definitions(${COMPONENT_LIB} PUBLIC "-DLOG_LOCAL_LEVEL=ESP_LOG_VERBOSE")
