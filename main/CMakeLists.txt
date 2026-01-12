idf_component_register(SRCS "softap_example_main.c"
                       PRIV_REQUIRES esp_wifi nvs_flash esp_driver_gpio esp_driver_uart
                       INCLUDE_DIRS ".")
