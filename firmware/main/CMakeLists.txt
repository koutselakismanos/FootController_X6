idf_component_register(
    SRCS "footswitch_controller.c" "main.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES esp_driver_gpio esp_driver_i2c nvs_flash
    )
