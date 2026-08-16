idf_component_register(
    SRCS
        "main.c"
        "can_handler.c"
        "sd_logger.c"
        "ble_nus.c"
        "vehicle_state.c"
        "web_interface.c"
        "oi_can.c"
        "oi_json.c"
    INCLUDE_DIRS "."
    REQUIRES
        driver
        esp_driver_twai
        bt
        nvs_flash
        fatfs
        sdmmc
        esp_timer
        esp_wifi
        esp_netif
        esp_event
        esp_http_server
        spiffs
        mdns
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/web_assets")
    spiffs_create_partition_image(spiffs "${CMAKE_CURRENT_SOURCE_DIR}/web_assets" FLASH_IN_PROJECT)
endif()
