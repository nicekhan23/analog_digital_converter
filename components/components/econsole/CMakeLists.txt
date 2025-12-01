idf_component_register(SRCS "econsole.c"
    INCLUDE_DIRS "."
    REQUIRES nvs_flash console esp_driver_uart fatfs esp_driver_usb_serial_jtag nvs_flash cmd_system cmd_wifi cmd_nvs)
