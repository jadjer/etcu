cmake_minimum_required(VERSION 3.30.5)

set(INCLUDE_DIRS
        include
)

set(SOURCES
        app_main.cpp

        src/App.cpp
        src/Telemetry.cpp
        src/Configuration.cpp
        src/Controller.cpp

        src/component/Motor.cpp
        src/component/Switch.cpp
        src/component/Throttle.cpp
        src/component/Indicator.cpp
        src/component/HourMeter.cpp
        src/component/ModeButton.cpp
        src/component/TwistPosition.cpp

        src/bluetooth/Bluetooth.cpp
        src/bluetooth/ServerCallback.cpp
        src/bluetooth/ConfigurationCharacteristicCallback.cpp

        src/bluetooth/ota/Updater.cpp
        src/bluetooth/ota/MessageHandler.cpp
        src/bluetooth/ota/UpdateCharacteristicCallback.cpp
)

idf_component_register(SRCS ${SOURCES} INCLUDE_DIRS ${INCLUDE_DIRS})
