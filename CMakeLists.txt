cmake_minimum_required(VERSION 3.30.5)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(
        ETCU
        DESCRIPTION "Electronic throttle control unit for Honda Varadero based on esp32-s3"
        HOMEPAGE_URL https://github.com/jadjer/etcu-esp32
        LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
