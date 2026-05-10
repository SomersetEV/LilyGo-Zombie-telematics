idf_component_register(
    SRCS
        "main.c"
        "can_handler.c"
        "sd_logger.c"
        "ble_nus.c"
        "vehicle_state.c"
    INCLUDE_DIRS "."
    REQUIRES
        driver
        esp_driver_twai
        bt
        nvs_flash
        fatfs
        sdmmc
        esp_timer
)
