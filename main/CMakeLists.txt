idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES driver bt nvs_flash esp_hw_support esp_driver_usb_serial_jtag mbedtls
)
