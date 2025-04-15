cmake_minimum_required(VERSION 3.28.1)

set(INCLUDE_DIRS
        include
)

set(SOURCES
        app_main.cpp

        src/App.cpp
        src/Motor.cpp
        src/Switch.cpp
        src/Throttle.cpp
        src/Indicator.cpp
        src/HourMeter.cpp
        src/Controller.cpp
        src/ModeButton.cpp
        src/TwistPosition.cpp

        src/bluetooth/Bluetooth.cpp
        src/bluetooth/ServerCallback.cpp
        src/bluetooth/ConfigurationCharacteristicCallback.cpp

        src/bluetooth/ota/Updater.cpp
        src/bluetooth/ota/MessageHandler.cpp
        src/bluetooth/ota/UpdateCharacteristicCallback.cpp

        src/configuration/Configuration.cpp
)

idf_component_register(SRCS ${SOURCES} INCLUDE_DIRS ${INCLUDE_DIRS})
