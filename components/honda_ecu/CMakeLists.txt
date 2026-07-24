cmake_minimum_required(VERSION 3.30.5)

set(INCLUDE_DIRS
        "include"
)

set(SOURCES
)

set(REQUIRES
)

idf_component_register(INCLUDE_DIRS ${INCLUDE_DIRS} SRCS ${SOURCES} REQUIRES ${REQUIRES})
