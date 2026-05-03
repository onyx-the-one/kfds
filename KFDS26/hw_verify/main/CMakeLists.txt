idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES
        led_strip
        esp_driver_gpio
        esp_driver_spi
        esp_rom
        freertos
        log
)
