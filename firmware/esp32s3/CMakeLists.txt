cmake_minimum_required(VERSION 3.16)

set(IDF_TARGET "esp32s3" CACHE STRING "ESP-IDF target")
set(NS2_ESP32S3_BOARD "n16r8" CACHE STRING "ESP32-S3 board profile")

set(NS2_ESP32S3_BASE_DEFAULTS "${CMAKE_CURRENT_LIST_DIR}/sdkconfig.defaults")
set(NS2_ESP32S3_BOARD_DEFAULTS "${CMAKE_CURRENT_LIST_DIR}/boards/${NS2_ESP32S3_BOARD}/sdkconfig.defaults")

if(NOT EXISTS "${NS2_ESP32S3_BOARD_DEFAULTS}")
    message(FATAL_ERROR "Unknown ESP32-S3 board profile '${NS2_ESP32S3_BOARD}'. Missing ${NS2_ESP32S3_BOARD_DEFAULTS}")
endif()

set(SDKCONFIG_DEFAULTS
    "${NS2_ESP32S3_BASE_DEFAULTS};${NS2_ESP32S3_BOARD_DEFAULTS}"
    CACHE STRING
    "ESP-IDF sdkconfig defaults"
)

# Keep generated sdkconfig files inside each build directory so board profiles
# can be built side by side without reusing stale flash/PSRAM settings.
set(SDKCONFIG
    "${CMAKE_BINARY_DIR}/sdkconfig"
    CACHE STRING
    "ESP-IDF sdkconfig path"
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
idf_build_set_property(COMPILE_DEFINITIONS
    "CFG_TUD_AUDIO=0"
    APPEND)
project(ns2pro_bridge_esp32s3_${NS2_ESP32S3_BOARD})
