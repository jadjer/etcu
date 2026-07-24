cmake_minimum_required(VERSION 3.30.5)

set(INCLUDE_DIRS
        "include"
)

set(SOURCES
        "app_main.cpp"
)

set(PRIV_REQUIRES
        esp_driver_uart
        esp_driver_ledc
        esp_driver_gpio
        esp_adc
        esp_timer
        nvs_flash
)

idf_component_register(INCLUDE_DIRS ${INCLUDE_DIRS} SRCS ${SOURCES} PRIV_REQUIRES ${PRIV_REQUIRES})
