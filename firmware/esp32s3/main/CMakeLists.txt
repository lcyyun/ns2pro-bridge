idf_component_register(
    SRCS
        "app_main.cpp"
        "ns2_board.cpp"
        "ns2_ble.cpp"
        "ns2_config.cpp"
        "ns2_gatt.cpp"
        "ns2_input.cpp"
        "ns2_protocol.cpp"
        "ns2_usb.cpp"
        "ns2_usb_uac1.cpp"
    INCLUDE_DIRS "."
    REQUIRES bt driver esp_tinyusb led_strip nvs_flash
)
