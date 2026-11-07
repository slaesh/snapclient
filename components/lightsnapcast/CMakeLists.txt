set(requires libbuffer json libmedian timefilter driver esp_timer)
if(CONFIG_SOC_WIFI_SUPPORTED)
    list(APPEND requires esp_wifi)
endif()

idf_component_register(SRCS "snapcast.c" "player.c" "snapcast_protocol_parser.c"
                       INCLUDE_DIRS "include"
                       REQUIRES ${requires})
