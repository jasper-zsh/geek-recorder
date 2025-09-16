idf_component_register(
    SRCS "app.c" "ui.c" "sdcard.c" "mic.c" "asr_ws.c" "lcd_text.c" "lvgl_port.c" "lvgl_fs_sd.c" "wifi_mgr.c" "http_server.c"
    INCLUDE_DIRS "." ".."
    REQUIRES esp_timer esp_lcd driver esp_psram sdmmc fatfs esp_http_server
             esp_wifi esp_netif nvs_flash esp_event esp_websocket_client lvgl
)
