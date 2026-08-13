cmake_minimum_required(VERSION 3.24.0)

set(INCLUDE_DIRS
        "include"
)

set(SOURCES
        "app_main.cpp"
)

idf_component_register(INCLUDE_DIRS ${INCLUDE_DIRS} SRCS ${SOURCES})
