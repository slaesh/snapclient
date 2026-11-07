set(srcs "network_interface.c")
set(requires driver esp_eth esp_netif esp_timer nvs_flash)

if(CONFIG_SNAPCLIENT_USE_INTERNAL_ETHERNET OR CONFIG_SNAPCLIENT_USE_SPI_ETHERNET)
    list(APPEND srcs "eth_interface.c")
endif()

if(CONFIG_SOC_WIFI_SUPPORTED)
    list(APPEND srcs "wifi_interface.c")
    list(APPEND requires esp_wifi improv_wifi)
endif()

idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES ${requires})

