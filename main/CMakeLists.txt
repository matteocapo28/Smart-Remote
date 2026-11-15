idf_component_register(SRCS "ir_nec_transceiver_main.c" "ir_nec_encoder.c" "lvgl_demo_ui.c"
                       PRIV_REQUIRES esp_driver_rmt
                       REQUIRES esp_driver_gpio
                       INCLUDE_DIRS ".")
