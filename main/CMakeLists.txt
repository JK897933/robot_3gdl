idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_driver_gpio esp_driver_i2c esp_driver_ledc esp_driver_uart log esp_timer nvs_flash esp_rom
)
