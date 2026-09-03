cmake_minimum_required(VERSION 3.24.0)

set(CMAKE_CXX_STANDARD 26)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(
        ETCU
        DESCRIPTION "Electronic throttle control unit for Honda Varadero based on esp32-s3"
        HOMEPAGE_URL https://github.com/jadjer/etcu
        VERSION 1.0.9
        LANGUAGES CXX
)
