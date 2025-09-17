#include <string.h>
#include <stdio.h>
#include <stdlib.h>

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
static lv_obj_t *s_top_bar = NULL;  // Top status bar (WiFi and IP)
static lv_obj_t *s_content_container = NULL;  // Scrollable content container
static lv_obj_t *s_content = NULL;  // Transcript label inside container
static lv_obj_t *s_bottom_bar = NULL;  // Bottom status bar (REC status and duration)
static lv_obj_t *s_asr_status = NULL;  // ASR status label (bottom right)
static const lv_font_t *s_font_default = NULL;
static const lv_font_t *s_font_chs = NULL;
#if LV_USE_FONT_LOADER
static lv_font_t *s_font_dynamic = NULL;
static bool s_lvgl_fs_registered = false;
#endif
static char s_transcript_buf[4096];
static char s_partial_buf[512];
static time_t s_rec_start_time = 0;  // Recording start time for duration calculation

static void ui_scroll_to_bottom_locked(void)
{
    if (!s_content_container) {
        return;
    }
    if (s_content) {
        lv_obj_update_layout(s_content);
    }
    lv_obj_update_layout(s_content_container);
    lv_coord_t bottom = lv_obj_get_scroll_bottom(s_content_container);
    lv_obj_scroll_to_y(s_content_container, bottom, LV_ANIM_OFF);
}

static void ui_refresh_transcript_locked(void)
{
    if (!(s_lvgl_ok && s_content)) {
        return;
    }
    size_t base_len = strlen(s_transcript_buf);
    size_t partial_len = strlen(s_partial_buf);
    size_t total_len = base_len + partial_len + 1;
    char *buf = malloc(total_len ? total_len : 1);
    if (!buf) {
        return;
    }
    if (base_len) memcpy(buf, s_transcript_buf, base_len);
    if (partial_len) memcpy(buf + base_len, s_partial_buf, partial_len);
    buf[base_len + partial_len] = '\0';
    lvgl_port_lock();
    lv_label_set_text(s_content, buf);
    // Auto-scroll to bottom
    ui_scroll_to_bottom_locked();
    lvgl_port_unlock();
    free(buf);
}

static void transcript_append_locked(const char *text)
{
    if (!text) {
        return;
    }
    size_t text_len = strlen(text);
    if (text_len == 0) {
        return;
    }
    size_t max_payload = sizeof(s_transcript_buf) - 2;
    if (text_len > max_payload) {
        text += text_len - max_payload;
        text_len = max_payload;
    }
    size_t len = strlen(s_transcript_buf);
    size_t need = text_len + 1; // newline + null terminator handled below
    if (len + need > sizeof(s_transcript_buf) - 1) {
        size_t overflow = len + need - (sizeof(s_transcript_buf) - 1);
        if (overflow > len) overflow = len;
        memmove(s_transcript_buf, s_transcript_buf + overflow, len - overflow + 1);
        len -= overflow;
    }
    memcpy(s_transcript_buf + len, text, text_len);
    len += text_len;
    s_transcript_buf[len++] = '\n';
    s_transcript_buf[len] = '\0';
}

static void ui_apply_font_locked(const lv_font_t *font)
{
    const lv_font_t *effective = font ? font : s_font_default;
    if (!effective) {
        effective = lv_font_default();
    }
    if (s_top_bar && effective) {
        lv_obj_set_style_text_font(s_top_bar, effective, LV_PART_MAIN);
    }
    if (s_bottom_bar && effective) {
        lv_obj_set_style_text_font(s_bottom_bar, effective, LV_PART_MAIN);
    }
    if (s_asr_status && effective) {
        lv_obj_set_style_text_font(s_asr_status, effective, LV_PART_MAIN);
    }
    if (s_content && effective) {
        lv_obj_set_style_text_font(s_content, effective, LV_PART_MAIN);
    }
    if (s_content_container) {
        const lv_font_t *st_font = effective;
        if (s_top_bar) {
            const lv_font_t *from_style = lv_obj_get_style_text_font(s_top_bar, LV_PART_MAIN);
            if (from_style) {
                st_font = from_style;
            }
        }
        int lh = st_font ? lv_font_get_line_height(st_font) : 16;
        if (lh <= 0) {
            lh = 16;
        }
        // Update content area position based on top bar height
        lv_obj_align_to(s_content_container, s_top_bar, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);
        lv_obj_set_width(s_content_container, LCD_HRES);
        // Calculate height: total height minus top bar height minus bottom bar height
        lv_obj_set_height(s_content_container, LCD_VRES - 2 * lh);
    }
}

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
        
        // Create top status bar (WiFi and IP address)
        s_top_bar = lv_label_create(lv_scr_act());
        lv_obj_set_width(s_top_bar, LCD_HRES);
        lv_obj_align(s_top_bar, LV_ALIGN_TOP_LEFT, 0, 0);
        lv_obj_set_style_text_color(s_top_bar, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_text_opa(s_top_bar, LV_OPA_COVER, 0);
        lv_label_set_text(s_top_bar, "Initializing...");
        
        // Create ASR status label (positioned at bottom right)
        s_asr_status = lv_label_create(lv_scr_act());
        lv_obj_set_style_text_color(s_asr_status, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_text_opa(s_asr_status, LV_OPA_COVER, 0);
        lv_label_set_text(s_asr_status, "ASR:OFF");
        lv_obj_align(s_asr_status, LV_ALIGN_BOTTOM_RIGHT, 0, 0);
        
        // Create bottom status bar (REC status and duration)
        s_bottom_bar = lv_label_create(lv_scr_act());
        lv_obj_set_width(s_bottom_bar, LCD_HRES - 80);  // Leave room for ASR status label
        lv_obj_align(s_bottom_bar, LV_ALIGN_BOTTOM_LEFT, 0, 0);
        lv_obj_set_style_text_color(s_bottom_bar, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_text_opa(s_bottom_bar, LV_OPA_COVER, 0);
        lv_label_set_text(s_bottom_bar, "Ready");
        
        // Create content container (scrollable area between top and bottom bars)
        s_content_container = lv_obj_create(lv_scr_act());
        lv_obj_remove_style_all(s_content_container);
        lv_obj_set_style_bg_opa(s_content_container, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(s_content_container, 0, 0);
        lv_obj_set_style_pad_all(s_content_container, 0, 0);
        lv_obj_set_width(s_content_container, LCD_HRES);
        lv_obj_align_to(s_content_container, s_top_bar, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);
        lv_obj_set_height(s_content_container, LCD_VRES - 40);  // Will be adjusted in ui_apply_font_locked
        lv_obj_set_scroll_dir(s_content_container, LV_DIR_VER);
        lv_obj_set_scrollbar_mode(s_content_container, LV_SCROLLBAR_MODE_AUTO);

        // Create transcript label inside container
        s_content = lv_label_create(s_content_container);
        lv_obj_set_width(s_content, LV_PCT(100));
        lv_label_set_long_mode(s_content, LV_LABEL_LONG_WRAP);
        lv_label_set_text(s_content, "");
        lv_obj_set_style_text_color(s_content, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_text_opa(s_content, LV_OPA_COVER, 0);
        lv_obj_set_style_pad_all(s_content, 0, 0);
        lv_obj_set_pos(s_content, 0, 0);

        s_font_default = lv_obj_get_style_text_font(s_top_bar, LV_PART_MAIN);
        if (!s_font_default) {
            s_font_default = lv_font_default();
        }
        s_font_chs = s_font_default;

        ui_apply_font_locked(s_font_chs);
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
    if (s_lvgl_ok && s_content) {
        lvgl_port_lock();
        const char *old = lv_label_get_text(s_content);
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
            lv_label_set_text(s_content, buf);
            // Auto-scroll to bottom
            ui_scroll_to_bottom_locked();
            free(buf);
        }
        lvgl_port_unlock();
    } else {
        lcd_text_println(text);
    }
    if (s_lcd_mutex) xSemaphoreGive(s_lcd_mutex);
}

void ui_transcript_set_partial(const char *text)
{
    if (!s_panel_handle) return;
    if (!text) text = "";
    if (s_lcd_mutex) xSemaphoreTake(s_lcd_mutex, portMAX_DELAY);
    snprintf(s_partial_buf, sizeof(s_partial_buf), "%s", text);
    if (s_lvgl_ok && s_content) {
        ui_refresh_transcript_locked();
    }
    if (s_lcd_mutex) xSemaphoreGive(s_lcd_mutex);
}

void ui_transcript_add_sentence(const char *text)
{
    if (!s_panel_handle || !text) return;
    if (s_lcd_mutex) xSemaphoreTake(s_lcd_mutex, portMAX_DELAY);
    transcript_append_locked(text);
    s_partial_buf[0] = '\0';
    if (s_lvgl_ok && s_content) {
        ui_refresh_transcript_locked();
    } else {
        lcd_text_println(text);
    }
    if (s_lcd_mutex) xSemaphoreGive(s_lcd_mutex);
}

static void ui_update_status_line_once(void)
{
    // Top bar: Wi‑Fi + IP (without ASR status)
    wifi_ap_record_t ap;
    bool wifi_ok = (esp_wifi_sta_get_ap_info(&ap) == ESP_OK);
    char wifi_part[48];
    if (wifi_ok) snprintf(wifi_part, sizeof(wifi_part), "WiFi:%s", (char*)ap.ssid);
    else snprintf(wifi_part, sizeof(wifi_part), "WiFi:--");

    char ip_part[24];
    esp_netif_ip_info_t ipi;
    esp_netif_t *sta = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (sta && esp_netif_get_ip_info(sta, &ipi) == ESP_OK && ipi.ip.addr != 0) {
        snprintf(ip_part, sizeof(ip_part), IPSTR, IP2STR(&ipi.ip));
    } else {
        strcpy(ip_part, "0.0.0.0");
    }

    char top_line[160];
    snprintf(top_line, sizeof(top_line), "%s | %s", wifi_part, ip_part);

    // ASR status (bottom right)
    const bool asr_conn = asr_ws_connected();
    const bool asr_strm = asr_ws_streaming();
    char asr_part[12];
    if (!asr_conn) snprintf(asr_part, sizeof(asr_part), "ASR:OFF");
    else if (asr_strm) snprintf(asr_part, sizeof(asr_part), "ASR:ON");
    else snprintf(asr_part, sizeof(asr_part), "ASR:RDY");

    // Bottom bar: REC status + duration
    bool is_recording = mic_is_recording();
    char rec_part[12]; 
    snprintf(rec_part, sizeof(rec_part), "REC:%s", is_recording ? "ON" : "OFF");
    
    char duration_part[16] = "";
    if (is_recording && s_rec_start_time > 0) {
        time_t now = time(NULL);
        int duration = now - s_rec_start_time;
        int hours = duration / 3600;
        int minutes = (duration % 3600) / 60;
        int seconds = duration % 60;
        if (hours > 0) {
            snprintf(duration_part, sizeof(duration_part), "%02d:%02d:%02d", hours, minutes, seconds);
        } else {
            snprintf(duration_part, sizeof(duration_part), "%02d:%02d", minutes, seconds);
        }
    } else if (!is_recording) {
        s_rec_start_time = 0;  // Reset start time when not recording
    }

    char bottom_line[80];
    if (strlen(duration_part) > 0) {
        snprintf(bottom_line, sizeof(bottom_line), "%s | %s", rec_part, duration_part);
    } else {
        snprintf(bottom_line, sizeof(bottom_line), "%s", rec_part);
    }

    if (s_lcd_mutex) xSemaphoreTake(s_lcd_mutex, portMAX_DELAY);
    if (s_lvgl_ok) {
        lvgl_port_lock();
        // Update top bar (WiFi and IP)
        if (s_top_bar) {
            lv_label_set_text(s_top_bar, top_line);
        }
        // Update ASR status (bottom right)
        if (s_asr_status) {
            lv_label_set_text(s_asr_status, asr_part);
        }
        // Update bottom bar
        if (s_bottom_bar) {
            lv_label_set_text(s_bottom_bar, bottom_line);
        }
        lvgl_port_unlock();
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
    ESP_LOGI(TAG, "Running post SD init tasks");
#if defined(CONFIG_GEEK_USE_LVGL)
    ESP_LOGI(TAG, "CONFIG_GEEK_USE_LVGL enabled");
#else
    ESP_LOGW(TAG, "CONFIG_GEEK_USE_LVGL not defined");
#endif
#ifdef LV_USE_FONT_LOADER
    ESP_LOGI(TAG, "LV_USE_FONT_LOADER=%d", LV_USE_FONT_LOADER);
#else
    ESP_LOGW(TAG, "LV_USE_FONT_LOADER not defined");
#endif
    ui_reload_font_from_sd();
}

bool ui_reload_font_from_sd(void)
{
#if defined(CONFIG_GEEK_USE_LVGL) && LV_USE_FONT_LOADER
    if (!s_lvgl_ok) {
        ESP_LOGW(TAG, "LVGL not ready; skip font reload");
        return false;
    }
    if (!s_lvgl_fs_registered) {
        ESP_LOGI(TAG, "Registering LVGL SD filesystem");
        lvgl_fs_sd_register();
        s_lvgl_fs_registered = true;
    }
    lvgl_port_lock();
    const char *font_path = "S:fonts/chs_16.bin";
    lv_fs_file_t font_file;
    lv_fs_res_t fs_res = lv_fs_open(&font_file, font_path, LV_FS_MODE_RD);
    if (fs_res == LV_FS_RES_OK) {
        uint32_t size = 0;
        if (lv_fs_seek(&font_file, 0, LV_FS_SEEK_END) == LV_FS_RES_OK &&
            lv_fs_tell(&font_file, &size) == LV_FS_RES_OK) {
            ESP_LOGI(TAG, "Font file found via LVGL FS: %u bytes", (unsigned)size);
        } else {
            ESP_LOGW(TAG, "Font file opened but size unknown (seek/tell failed)");
        }
        lv_fs_close(&font_file);
    } else {
        ESP_LOGE(TAG, "Font file open failed (%d)", fs_res);
    }
    ESP_LOGI(TAG, "Loading LVGL font from %s", font_path);
    lv_font_t *new_font = lv_font_load(font_path);
    if (!new_font) {
        if (s_font_dynamic) {
            lv_font_free(s_font_dynamic);
            s_font_dynamic = NULL;
        }
        s_font_chs = s_font_default;
        ui_apply_font_locked(s_font_chs);
        lvgl_port_unlock();
        ESP_LOGW(TAG, "Font load failed; using default font");
        return false;
    }
    lv_font_t *old_font = s_font_dynamic;
    s_font_dynamic = new_font;
    s_font_chs = s_font_dynamic;
    ui_apply_font_locked(s_font_chs);
    if (old_font) {
        lv_font_free(old_font);
    }
    lvgl_port_unlock();
    ESP_LOGI(TAG, "Loaded LVGL font from SD");
    return true;
#else
    ESP_LOGW(TAG, "LVGL font loader disabled; skip font reload");
    return false;
#endif
}

void ui_record_started(void)
{
    if (s_lvgl_ok) {
        s_rec_start_time = time(NULL);
    }
}
