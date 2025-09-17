#include <stdio.h>
#include <string.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_sntp.h"
#include "nvs_flash.h"
#include "driver/gpio.h"

#include "wifi_mgr.h"
#include "http_server.h"
#include "ui.h"
#include "sdcard.h"
#include "mic.h"
#include "asr_ws.h"

static const char *TAG = "app";

// BOOT key toggles record; long press enters provisioning
static void boot_key_task(void *pvParameters)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << 0),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    TickType_t press_start_time = 0;
    bool was_pressed = false;
    while (1) {
        bool is_pressed = (gpio_get_level(0) == 0);
        if (is_pressed && !was_pressed) {
            press_start_time = xTaskGetTickCount();
        } else if (!is_pressed && was_pressed) {
            TickType_t press_duration = xTaskGetTickCount() - press_start_time;
            uint32_t press_duration_ms = press_duration * portTICK_PERIOD_MS;
            if (press_duration_ms >= 3000) {
                wifi_mgr_enter_provisioning();
            } else {
                static bool rec = false;
                rec = !rec;
                mic_set_record_request(rec);
                ESP_LOGI(TAG, "Recording %s", rec ? "START" : "STOP");
            }
        }
        was_pressed = is_pressed;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Geek Recorder boot");

    xTaskCreatePinnedToCore(boot_key_task, "boot_key", 4096, NULL, 5, NULL, tskNO_AFFINITY);

    ESP_ERROR_CHECK(nvs_flash_init());

    /* Stage 1: peripherals */
    ESP_LOGI(TAG, "Initializing peripherals");
    bool sd_ok = sdcard_mount();
    if (!sd_ok) {
        ESP_LOGW(TAG, "SD card not mounted");
    }

    bool mic_ok = false;
    if (sd_ok) {
        if (mic_init() == ESP_OK) {
            mic_ok = true;
        } else {
            ESP_LOGE(TAG, "Mic init failed");
        }
    } else {
        ESP_LOGW(TAG, "Skip mic init (SD unavailable)");
    }

    /* Stage 2: UI */
    ESP_LOGI(TAG, "Initializing UI");
    ui_init();
    if (sd_ok) {
        ui_post_sd_init();
    }

    /* Stage 3: network */
    ESP_LOGI(TAG, "Initializing network");
    wifi_mgr_init();
    ui_start_status_task();

    vTaskDelay(pdMS_TO_TICKS(500));
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_init();

    // Wait for time (best-effort)
    int retry = 0;
    while (esp_sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < 10) {
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    /* Stage 4: other modules */
    if (mic_ok) {
        mic_start_task();
    }
    http_server_start(NULL);

    ESP_LOGI(TAG, "System initialized");
}
