#include <string.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_netif.h"
#include "esp_wifi.h"

#include "lcd_text.h"
#include "lvgl.h"
#include "lvgl_port.h"

#include "ui.h"
#include "mic.h"
#include "asr_ws.h"
#include <time.h>

static const char *TAG = "ui";

// LCD config from Kconfig
#define LCD_HOST_NUM      CONFIG_GEEK_LCD_HOST
#define LCD_GPIO_DC       CONFIG_GEEK_LCD_GPIO_DC
#define LCD_GPIO_CS       CONFIG_GEEK_LCD_GPIO_CS
#define LCD_GPIO_SCLK     CONFIG_GEEK_LCD_GPIO_SCLK
#define LCD_GPIO_MOSI     CONFIG_GEEK_LCD_GPIO_MOSI
#define LCD_GPIO_RST      CONFIG_GEEK_LCD_GPIO_RST
#define LCD_GPIO_BL       CONFIG_GEEK_LCD_GPIO_BL
#define LCD_HRES          CONFIG_GEEK_LCD_HRES
#define LCD_VRES          CONFIG_GEEK_LCD_VRES

static esp_lcd_panel_handle_t s_panel_handle = NULL;
static SemaphoreHandle_t s_lcd_mutex = NULL;
static bool s_lvgl_ok = false;
static lv_obj_t *s_label = NULL;
static lv_obj_t *s_status_label = NULL;
#if LV_USE_FONT_LOADER
static const lv_font_t *s_font_chs = NULL;
#endif

static void set_gpio_output(int gpio, int level)
{
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << gpio,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
    gpio_set_level(gpio, level);
}

static esp_err_t lcd_init_internal(void)
{
    ESP_LOGI(TAG, "Init LCD SPI bus and panel");

    set_gpio_output(LCD_GPIO_BL, 1);

    spi_bus_config_t buscfg = {
        .mosi_io_num = LCD_GPIO_MOSI,
        .miso_io_num = -1,
        .sclk_io_num = LCD_GPIO_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_HRES * LCD_VRES * 2 + 8,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST_NUM == 2 ? SPI2_HOST : SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));

    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = LCD_GPIO_DC,
        .cs_gpio_num = LCD_GPIO_CS,
        .pclk_hz = 20 * 1000 * 1000,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 10,
        .on_color_trans_done = NULL,
        .user_ctx = NULL,
        .flags = { .dc_low_on_data = 0 },
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(LCD_HOST_NUM == 2 ? SPI2_HOST : SPI3_HOST, &io_config, &io_handle));

#ifdef CONFIG_GEEK_LCD_DRIVER_ST7789
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = LCD_GPIO_RST,
        .color_space = ESP_LCD_COLOR_SPACE_RGB,
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &s_panel_handle));
#else
#error "Only ST7789 currently supported"
#endif

    ESP_ERROR_CHECK(esp_lcd_panel_reset(s_panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(s_panel_handle));
    esp_lcd_panel_swap_xy(s_panel_handle, true);
    esp_lcd_panel_mirror(s_panel_handle, true, false);
    esp_lcd_panel_set_gap(s_panel_handle, 40, 53);
    esp_lcd_panel_invert_color(s_panel_handle, true);
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(s_panel_handle, true));

    s_lcd_mutex = xSemaphoreCreateMutex();

#ifdef CONFIG_GEEK_USE_LVGL
    s_lvgl_ok = lvgl_port_init(s_panel_handle, LCD_HRES, LCD_VRES, CONFIG_GEEK_LVGL_TASK_STACK);
    if (s_lvgl_ok) {
        lvgl_port_lock();
        lv_obj_t *scr = lv_scr_act();
        lv_obj_set_style_bg_color(scr, lv_color_hex(0x000000), 0);
        lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
        s_status_label = lv_label_create(lv_scr_act());
        lv_obj_set_width(s_status_label, LCD_HRES);
        lv_obj_align(s_status_label, LV_ALIGN_TOP_LEFT, 0, 0);
        lv_obj_set_style_text_color(s_status_label, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_text_opa(s_status_label, LV_OPA_COVER, 0);
        s_label = lv_label_create(lv_scr_act());
        lv_obj_set_width(s_label, LCD_HRES);
        lv_label_set_long_mode(s_label, LV_LABEL_LONG_WRAP);
        const lv_font_t *st_font = lv_obj_get_style_text_font(s_status_label, LV_PART_MAIN);
        int lh = st_font ? lv_font_get_line_height(st_font) : 16;
        if (lh <= 0) lh = 16;
        lv_obj_align(s_label, LV_ALIGN_TOP_LEFT, 0, lh + 2);
        lv_label_set_text(s_label, "");
        lvgl_port_unlock();
    }
#endif
    if (!s_lvgl_ok) {
        lcd_text_cfg_t cfg = { .cols = LCD_HRES/8, .rows = LCD_VRES/8, .fg = 0xFFFF, .bg = 0x0000 };
        lcd_text_init(s_panel_handle, LCD_HRES, LCD_VRES, &cfg);
    }
    return ESP_OK;
}

void ui_init(void)
{
    lcd_init_internal();
}

void ui_show_text(const char *text)
{
    if (!s_panel_handle) return;
    if (s_lcd_mutex) xSemaphoreTake(s_lcd_mutex, portMAX_DELAY);
    if (s_lvgl_ok && s_label) {
        lvgl_port_lock();
        const char *old = lv_label_get_text(s_label);
        size_t old_len = old ? strlen(old) : 0;
        size_t add_len = strlen(text) + 1;
        size_t max_len = 4096;
        char *buf = malloc(old_len + add_len + 2);
        if (buf) {
            strcpy(buf, old ? old : "");
            strcat(buf, text);
            strcat(buf, "\n");
            if (strlen(buf) > max_len) {
                size_t diff = strlen(buf) - max_len;
                memmove(buf, buf + diff, strlen(buf) - diff + 1);
            }
            lv_label_set_text(s_label, buf);
            free(buf);
        }
        lvgl_port_unlock();
    } else {
        lcd_text_println(text);
    }
    if (s_lcd_mutex) xSemaphoreGive(s_lcd_mutex);
}

static void ui_update_status_line_once(void)
{
    // Wi‑Fi + IP
    wifi_ap_record_t ap;
    bool wifi_ok = (esp_wifi_sta_get_ap_info(&ap) == ESP_OK);
    char wifi_part[48];
    if (wifi_ok) snprintf(wifi_part, sizeof(wifi_part), "WiFi:%s %ddBm", (char*)ap.ssid, ap.rssi);
    else snprintf(wifi_part, sizeof(wifi_part), "WiFi:--");

    char ip_part[24];
    esp_netif_ip_info_t ipi;
    esp_netif_t *sta = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (sta && esp_netif_get_ip_info(sta, &ipi) == ESP_OK && ipi.ip.addr != 0) {
        snprintf(ip_part, sizeof(ip_part), IPSTR, IP2STR(&ipi.ip));
    } else {
        strcpy(ip_part, "0.0.0.0");
    }

    char rec_part[12]; snprintf(rec_part, sizeof(rec_part), "REC:%s", mic_is_recording() ? "ON" : "OFF");
    const bool asr_conn = asr_ws_connected();
    const bool asr_strm = asr_ws_streaming();
    char asr_part[12];
    if (!asr_conn) snprintf(asr_part, sizeof(asr_part), "ASR:OFF");
    else if (asr_strm) snprintf(asr_part, sizeof(asr_part), "ASR:ON");
    else snprintf(asr_part, sizeof(asr_part), "ASR:RDY");

    time_t now = time(NULL); struct tm tm; localtime_r(&now, &tm);
    char tbuf[12]; snprintf(tbuf, sizeof(tbuf), "%02d:%02d:%02d", tm.tm_hour, tm.tm_min, tm.tm_sec);

    char line[160];
    snprintf(line, sizeof(line), "%s  |  IP:%s  |  %s  |  %s  |  %s",
             wifi_part, ip_part, rec_part, asr_part, tbuf);

    if (s_lcd_mutex) xSemaphoreTake(s_lcd_mutex, portMAX_DELAY);
    if (s_lvgl_ok && s_status_label) {
        lvgl_port_lock();
        lv_label_set_text(s_status_label, line);
        lvgl_port_unlock();
    } else {
        lcd_text_set_status_line(line);
    }
    if (s_lcd_mutex) xSemaphoreGive(s_lcd_mutex);
}

void ui_update_status_line(void)
{
    ui_update_status_line_once();
}

static void status_task(void *arg)
{
    while (1) {
        ui_update_status_line_once();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void ui_start_status_task(void)
{
    xTaskCreatePinnedToCore(status_task, "ui_status", 4096, NULL, 4, NULL, tskNO_AFFINITY);
}

void ui_post_sd_init(void)
{
#ifdef CONFIG_GEEK_USE_LVGL
    if (s_lvgl_ok) {
        lvgl_fs_sd_register();
        lvgl_port_lock();
#if LV_USE_FONT_LOADER
        s_font_chs = lv_font_load("S:/fonts/chs_16.bin");
        if (s_font_chs && s_label) {
            lv_obj_set_style_text_font(s_label, s_font_chs, LV_PART_MAIN);
            ESP_LOGI(TAG, "Loaded LVGL font from SD");
        } else if (!s_font_chs) {
            ESP_LOGW(TAG, "Failed to load font from SD; using default");
        }
#else
        ESP_LOGW(TAG, "LV_USE_FONT_LOADER=0; cannot load font from SD");
#endif
        lvgl_port_unlock();
    }
#endif
}
