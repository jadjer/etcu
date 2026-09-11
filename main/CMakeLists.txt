cmake_minimum_required(VERSION 3.24.0)

string(TIMESTAMP BUILD_DATE "%Y-%m-%d")

set(BOARD_VERSION "v1.0")
set(INCLUDE_DIRS "include")
set(SOURCES "app_main.cpp")

set(REQUIRES
        "esp_adc"
        "esp_driver_uart"
        "app_update"
)

idf_component_register(INCLUDE_DIRS ${INCLUDE_DIRS} SRCS ${SOURCES} REQUIRES ${REQUIRES})

target_compile_definitions(${COMPONENT_LIB} PRIVATE
        -DFW_VERSION="${CMAKE_PROJECT_VERSION}"
        -DBOARD_VERSION="${BOARD_VERSION}"
        -DBUILD_DATE="${BUILD_DATE}"
)
