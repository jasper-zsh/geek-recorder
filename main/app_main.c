#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <dirent.h>
#include <time.h>
#include <ctype.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "esp_event.h"
#include "esp_system.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_netif_types.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_sntp.h"

// WiFi断开原因码定义
#define WIFI_REASON_NO_AP_FOUND          201
#define WIFI_REASON_AUTH_FAIL            202
#define WIFI_REASON_ASSOC_LEAVE          203

#include "driver/gpio.h"
#include "driver/spi_master.h"

#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "lcd_text.h"
#include "lvgl.h"
#include "lvgl_port.h"

#include "driver/sdmmc_host.h"
#include "driver/sdmmc_defs.h"
#include "sdmmc_cmd.h"
#include "esp_vfs_fat.h"

#include "esp_http_server.h"
#include "esp_wifi.h"
#include "esp_netif.h"

#include "esp_websocket_client.h"
#include "esp_crt_bundle.h"
#include "wifi_mgr.h"
#include "http_server.h"

#include "driver/i2s_std.h"
#include "driver/i2s_pdm.h"
#include "esp_heap_caps.h"

static const char *TAG = "geek_recorder";

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

// MIC config
#define MIC_GPIO_CLK      CONFIG_GEEK_MIC_PDM_CLK
#define MIC_GPIO_DATA     CONFIG_GEEK_MIC_PDM_DATA
#define MIC_GPIO_SEL      CONFIG_GEEK_MIC_SEL_GPIO
#define MIC_SAMPLE_RATE   CONFIG_GEEK_MIC_SAMPLE_RATE

// SDMMC config
#define SD_CLK_GPIO       CONFIG_GEEK_SDMMC_CLK
#define SD_CMD_GPIO       CONFIG_GEEK_SDMMC_CMD
#define SD_D0_GPIO        CONFIG_GEEK_SDMMC_D0
#define SD_D1_GPIO        CONFIG_GEEK_SDMMC_D1
#define SD_D2_GPIO        CONFIG_GEEK_SDMMC_D2
#define SD_D3_GPIO        CONFIG_GEEK_SDMMC_D3

// HTTP
#define HTTP_PORT         CONFIG_GEEK_HTTP_PORT
// Provisioning AP password from Kconfig (may be empty)
#ifndef CONFIG_GEEK_AP_PASSWORD
#define CONFIG_GEEK_AP_PASSWORD ""
#endif
#define AP_PASSWORD       CONFIG_GEEK_AP_PASSWORD

// NVS keys
#define NVS_NS_WIFI   "wifi"
#define NVS_KEY_SSID  "ssid"
#define NVS_KEY_PASS  "pass"
#define NVS_NS_APP    "app"
#define NVS_KEY_API   "api_key"

// Globals
static sdmmc_card_t *s_card = NULL;
static i2s_chan_handle_t s_pdm_rx_chan = NULL;
static esp_lcd_panel_handle_t s_panel_handle = NULL;
static SemaphoreHandle_t s_lcd_mutex = NULL;
static bool s_lvgl_ok = false;
static lv_obj_t *s_label = NULL;
static lv_obj_t *s_status_label = NULL;
static const lv_font_t *s_font_chs = NULL;
static esp_websocket_client_handle_t s_ws = NULL;
static volatile bool s_ws_connected = false;
static volatile bool s_ws_task_started = false;
static char s_ws_task_id[64] = {0};
static char s_current_rec_base[128] = {0}; // e.g., /sdcard/rec-YYYYMMDD-HHMMSS
static FILE *s_transcript_fp = NULL;
static volatile bool s_recording_on = false;

// Build and push a single-line status string to the UI top bar
static void ui_update_status_line(void)
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

    // Record
    char rec_part[12]; snprintf(rec_part, sizeof(rec_part), "REC:%s", s_recording_on ? "ON" : "OFF");

    // ASR
    char asr_part[12]; snprintf(asr_part, sizeof(asr_part), "ASR:%s", s_ws_connected ? "ON" : "OFF");

    // Time
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

// Forward decls
static esp_err_t lcd_init(void);
static esp_err_t sdcard_mount(void);
static esp_err_t mic_init(void);
static void mic_capture_task(void *arg);
static void show_text(const char *text);
static void ws_task(void *arg);
static void ws_start_if_ready(void);

// WiFi连接状态检查任务
static void wifi_monitor_task(void *pvParameters)
{
    while (1) {
        wifi_ap_record_t ap_info;
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
            ESP_LOGI(TAG, "WiFi connected to: %s, RSSI: %d", ap_info.ssid, ap_info.rssi);
        } else {
            ESP_LOGW(TAG, "WiFi not connected");
        }
        vTaskDelay(pdMS_TO_TICKS(30000)); // 每30秒检查一次
    }
}

// BOOT按键检测任务
static volatile bool s_record_request = false;

static void boot_key_task(void *pvParameters)
{
    ESP_LOGI(TAG, "BOOT key task started, monitoring GPIO0");
    
    // 配置BOOT按键GPIO
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << 0),  // GPIO0通常为BOOT键
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    
    TickType_t press_start_time = 0;
    bool was_pressed = false;
    
    ESP_LOGI(TAG, "BOOT key task initialized, starting monitoring loop");
    
    // 添加GPIO0状态测试
    int test_count = 0;
    
    while (1) {
        // 读取BOOT按键状态（低电平表示按下）
        bool is_pressed = (gpio_get_level(0) == 0);
        
        // 添加调试日志
        static bool last_state = true; // 默认未按下
        if (is_pressed != last_state) {
            ESP_LOGI(TAG, "BOOT key state changed: %s", is_pressed ? "PRESSED" : "RELEASED");
            last_state = is_pressed;
        }
        
        if (is_pressed && !was_pressed) {
            // 按键按下
            press_start_time = xTaskGetTickCount();
            ESP_LOGI(TAG, "BOOT key pressed at tick %lu", (unsigned long)press_start_time);
        } else if (!is_pressed && was_pressed) {
            // 按键释放
            TickType_t press_duration = xTaskGetTickCount() - press_start_time;
            // 转换为毫秒
            uint32_t press_duration_ms = press_duration * portTICK_PERIOD_MS;
            
            ESP_LOGI(TAG, "BOOT key released, pressed for %lu ms", (unsigned long)press_duration_ms);
            
            if (press_duration_ms >= 3000) {
                // 长按≥3秒：切换为配网模式（不清除任何设置）
                ESP_LOGI(TAG, "BOOT long press: switch to provisioning mode");
                wifi_mgr_enter_provisioning();
            } else {
                // 短按：开始/停止录音
                s_record_request = !s_record_request;
                ESP_LOGI(TAG, "BOOT short press: recording %s", s_record_request ? "START" : "STOP");
            }
        }
        
        was_pressed = is_pressed;
        vTaskDelay(pdMS_TO_TICKS(100)); // 每100ms检查一次
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

static esp_err_t lcd_init(void)
{
    ESP_LOGI(TAG, "Init LCD SPI bus and panel");

    // Configure backlight pin
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
        .pclk_hz = 20 * 1000 * 1000, // lower to improve signal integrity
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 10,
        .on_color_trans_done = NULL,
        .user_ctx = NULL,
        .flags = {
            .dc_low_on_data = 0,
        },
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
    // Common ST7789 240x135 tuning: swap XY + mirror X + set gaps
    esp_lcd_panel_swap_xy(s_panel_handle, true);
    esp_lcd_panel_mirror(s_panel_handle, true, false);
    esp_lcd_panel_set_gap(s_panel_handle, 40, 53);
    esp_lcd_panel_invert_color(s_panel_handle, true);
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(s_panel_handle, true));

    s_lcd_mutex = xSemaphoreCreateMutex();
    // Prefer LVGL for CJK; fallback to ascii text console
#ifdef CONFIG_GEEK_USE_LVGL
    s_lvgl_ok = lvgl_port_init(s_panel_handle, LCD_HRES, LCD_VRES, CONFIG_GEEK_LVGL_TASK_STACK);
    if (s_lvgl_ok) {
        lvgl_port_lock();
        // Set a solid black background to guarantee contrast
        lv_obj_t *scr = lv_scr_act();
        lv_obj_set_style_bg_color(scr, lv_color_hex(0x000000), 0);
        lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
        // Status label at the very top
        s_status_label = lv_label_create(lv_scr_act());
        lv_obj_set_width(s_status_label, LCD_HRES);
        lv_obj_align(s_status_label, LV_ALIGN_TOP_LEFT, 0, 0);
        lv_obj_set_style_text_color(s_status_label, lv_color_hex(0xFFFFFF), 0);
        // Slightly brighter status text for readability
        lv_obj_set_style_text_opa(s_status_label, LV_OPA_COVER, 0);
        // Main log label placed just below status bar using real font line-height
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

static void rgb565_fill(uint16_t *buf, size_t n, uint8_t r, uint8_t g, uint8_t b)
{
    uint16_t c = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
    for (size_t i = 0; i < n; ++i) buf[i] = c;
}

static void show_text(const char *text)
{
    if (!s_panel_handle) return;
    if (s_lcd_mutex) xSemaphoreTake(s_lcd_mutex, portMAX_DELAY);
    if (s_lvgl_ok && s_label) {
        lvgl_port_lock();
        // Append and trim if too long
        const char *old = lv_label_get_text(s_label);
        size_t old_len = old ? strlen(old) : 0;
        size_t add_len = strlen(text) + 1; // plus newline
        size_t max_len = 4096; // simple cap
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

static uint32_t urand32(void) { return esp_random(); }
static void gen_uuid_like(char *out, size_t out_sz)
{
    const char *hex = "0123456789abcdef";
    int idxs[36] = {8,13,18,23};
    size_t j=0;
    for (int i=0;i<36 && j+1<out_sz;i++){
        bool dash = (i==8||i==13||i==18||i==23);
        if (dash){ out[j++]='-'; continue; }
        uint32_t r = urand32();
        out[j++] = hex[(r>>((i&3)*4))&0xF];
    }
    out[j]=0;
}

static esp_err_t sdcard_mount(void)
{
    ESP_LOGI(TAG, "Mount SD card (SDMMC 4-bit)");
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.max_freq_khz = SDMMC_FREQ_DEFAULT;
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.clk = SD_CLK_GPIO;
    slot_config.cmd = SD_CMD_GPIO;
    slot_config.d0  = SD_D0_GPIO;
    slot_config.d1  = SD_D1_GPIO;
    slot_config.d2  = SD_D2_GPIO;
    slot_config.d3  = SD_D3_GPIO;
    slot_config.width = 4;

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 8,
        .allocation_unit_size = 16 * 1024,
    };

    esp_err_t ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &s_card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount SD card: %s", esp_err_to_name(ret));
        return ret;
    }
    sdmmc_card_print_info(stdout, s_card);
    return ESP_OK;
}

static esp_err_t mic_init(void)
{
    ESP_LOGI(TAG, "Init PDM mic: clk=%d, data=%d, sel=%d, %d Hz", MIC_GPIO_CLK, MIC_GPIO_DATA, MIC_GPIO_SEL, MIC_SAMPLE_RATE);
    // Set SEL to choose channel (left/right depending on mic). Use low by default.
    set_gpio_output(MIC_GPIO_SEL, 0);

    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, &s_pdm_rx_chan));

    i2s_pdm_rx_config_t pdm_rx_cfg = {
        .clk_cfg = I2S_PDM_RX_CLK_DEFAULT_CONFIG(MIC_SAMPLE_RATE),
        .slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .clk = MIC_GPIO_CLK,
            .din = MIC_GPIO_DATA,
            .invert_flags = {
                .clk_inv = false,
            }
        }
    };
    ESP_ERROR_CHECK(i2s_channel_init_pdm_rx_mode(s_pdm_rx_chan, &pdm_rx_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(s_pdm_rx_chan));
    return ESP_OK;
}

// Simple WAV header helper
typedef struct __attribute__((packed)) {
    char riff[4];
    uint32_t size;
    char wave[4];
    char fmt[4];
    uint32_t fmt_size;
    uint16_t audio_format;
    uint16_t num_channels;
    uint32_t sample_rate;
    uint32_t byte_rate;
    uint16_t block_align;
    uint16_t bits_per_sample;
    char data[4];
    uint32_t data_size;
} wav_header_t;

static void write_wav_header(FILE *f, uint32_t sample_rate)
{
    wav_header_t h = {0};
    memcpy(h.riff, "RIFF", 4);
    memcpy(h.wave, "WAVE", 4);
    memcpy(h.fmt,  "fmt ", 4);
    memcpy(h.data, "data", 4);
    h.fmt_size = 16;
    h.audio_format = 1;
    h.num_channels = 1;
    h.sample_rate = sample_rate;
    h.bits_per_sample = 16;
    h.byte_rate = sample_rate * h.num_channels * (h.bits_per_sample/8);
    h.block_align = h.num_channels * (h.bits_per_sample/8);
    h.data_size = 0; // fixup at close
    h.size = 36 + h.data_size;
    fwrite(&h, 1, sizeof(h), f);
}

static void fixup_wav_header(FILE *f)
{
    long end = ftell(f);
    if (end < (long)sizeof(wav_header_t)) return;
    uint32_t data_size = (uint32_t)(end - sizeof(wav_header_t));
    fseek(f, 4, SEEK_SET);
    uint32_t riff_size = 36 + data_size;
    fwrite(&riff_size, 1, 4, f);
    fseek(f, 40, SEEK_SET);
    fwrite(&data_size, 1, 4, f);
    fseek(f, end, SEEK_SET);
}

static FILE *s_wav_fp = NULL;

static void open_new_wav_if_needed(void)
{
    if (s_wav_fp || !s_record_request) return;
    // Prepare path
    char path[128];
    time_t now = time(NULL);
    struct tm tm;
    localtime_r(&now, &tm);
    strftime(s_current_rec_base, sizeof(s_current_rec_base), "/sdcard/rec-%Y%m%d-%H%M%S", &tm);
    strlcpy(path, s_current_rec_base, sizeof(path));
    strlcat(path, ".wav", sizeof(path));
    s_wav_fp = fopen(path, "wb");
    if (!s_wav_fp) {
        ESP_LOGE(TAG, "Failed to open %s", path);
        return;
    }
    write_wav_header(s_wav_fp, MIC_SAMPLE_RATE);
    ESP_LOGI(TAG, "Recording to %s", path);
    s_recording_on = true;
}

static void close_wav_if_open(void)
{
    if (!s_wav_fp) return;
    fixup_wav_header(s_wav_fp);
    fclose(s_wav_fp);
    s_wav_fp = NULL;
    s_recording_on = false;
    ESP_LOGI(TAG, "Recording stopped");
}

static void mic_capture_task(void *arg)
{
    ESP_LOGI(TAG, "Mic capture task started");
    const size_t buf_bytes = 2048;
    uint8_t *buf = heap_caps_malloc(buf_bytes, MALLOC_CAP_DMA);
    if (!buf) {
        ESP_LOGE(TAG, "No mem for mic buffer");
        vTaskDelete(NULL);
        return;
    }

    while (1) {
        size_t bytes_read = 0;
        esp_err_t ret = i2s_channel_read(s_pdm_rx_chan, buf, buf_bytes, &bytes_read, pdMS_TO_TICKS(1000));
        // Handle recording state transitions
        if (s_record_request && !s_wav_fp) {
            open_new_wav_if_needed();
        } else if (!s_record_request && s_wav_fp) {
            close_wav_if_open();
        }
        if (ret == ESP_OK && s_wav_fp) {
            fwrite(buf, 1, bytes_read, s_wav_fp);
        }
        // Stream to WS if task started
        if (ret == ESP_OK && s_ws && s_ws_connected && s_ws_task_started) {
            esp_websocket_client_send_bin(s_ws, (const char *)buf, bytes_read, portMAX_DELAY);
        }
    }
}

static void status_task(void *arg)
{
    while (1) {
        ui_update_status_line();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Minimal HTTP server: provisioning + file list/download/delete
static esp_err_t http_send_text(httpd_req_t *req, const char *text)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    return httpd_resp_sendstr(req, text);
}

static void html_escape(char *dst, size_t dst_sz, const char *src)
{
    size_t di = 0;
    for (const unsigned char *p = (const unsigned char *)src; *p && di + 6 < dst_sz; ++p) {
        if (*p == '&')      di += snprintf(dst+di, dst_sz-di, "&amp;");
        else if (*p == '<') di += snprintf(dst+di, dst_sz-di, "&lt;");
        else if (*p == '>') di += snprintf(dst+di, dst_sz-di, "&gt;");
        else if (*p == '"')di += snprintf(dst+di, dst_sz-di, "&quot;");
        else dst[di++] = *p;
    }
    dst[di] = 0;
}

/* Moved to http_server.c
static esp_err_t root_get_handler(httpd_req_t *req)
{
    nvs_handle_t n;
    char ssid[33] = {0};
    char pass[65] = {0};
    char api_key[129] = {0};
    size_t len;
    bool has_wifi = false, has_api = false;
    
    // 检查WiFi配置的有效性
    if (nvs_open(NVS_NS_WIFI, NVS_READONLY, &n) == ESP_OK) {
        len = sizeof(ssid);
        if (nvs_get_str(n, NVS_KEY_SSID, ssid, &len) == ESP_OK && strlen(ssid) > 0) {
            len = sizeof(pass);
            if (nvs_get_str(n, NVS_KEY_PASS, pass, &len) == ESP_OK) {
                // 检查密码长度是否合法（WPA2要求至少8位）
                if (strlen(pass) >= 8 || strlen(pass) == 0) {  // 0表示开放网络
                    has_wifi = true;
                }
            }
        }
        nvs_close(n);
    }
    
    if (nvs_open(NVS_NS_APP, NVS_READONLY, &n) == ESP_OK) {
        len = sizeof(api_key);
        has_api = (nvs_get_str(n, NVS_KEY_API, api_key, &len) == ESP_OK);
        nvs_close(n);
    }

    // Stream HTML in chunks to avoid large buffers
    httpd_resp_sendstr_chunk(req, "<html><head><meta charset='utf-8'><title>Geek Recorder</title></head><body>");
    httpd_resp_sendstr_chunk(req, "<h2>Geek Recorder</h2>");
    httpd_resp_sendstr_chunk(req, "<p>Wi-Fi: ");
    httpd_resp_sendstr_chunk(req, has_wifi ? "configured" : "not configured");
    httpd_resp_sendstr_chunk(req, "</p><p>API Key: ");
    httpd_resp_sendstr_chunk(req, has_api ? "configured" : "not configured");
    httpd_resp_sendstr_chunk(req, "</p><h3>Provision</h3>");
    httpd_resp_sendstr_chunk(req, "<form method='POST' action='/provision'>SSID: <input id='ssid' name='ssid' maxlength='32' list='ssidlist' required> ");
    httpd_resp_sendstr_chunk(req, "Password: <input name='pass' type='password' maxlength='64' minlength='8' placeholder='>=8 chars for WPA/WPA2'> ");
    httpd_resp_sendstr_chunk(req, "API Key: <input name='api' maxlength='128'>");
    httpd_resp_sendstr_chunk(req, "<button type='submit'>Save</button> <button type='button' id='scan'>Scan Wi-Fi</button></form>");
    httpd_resp_sendstr_chunk(req, "<datalist id='ssidlist'></datalist>");
    httpd_resp_sendstr_chunk(req, "<div><select id='ssidsel' size='8' style='width:100%;max-width:340px;'></select></div>");
    httpd_resp_sendstr_chunk(req, "<script>\nconst btn=document.getElementById('scan');\nconst list=document.getElementById('ssidlist');\nconst sel=document.getElementById('ssidsel');\nconst ssid=document.getElementById('ssid');\nfunction fillLists(data){list.innerHTML='';sel.innerHTML='';let seen={};data.sort((a,b)=>b.rssi-a.rssi).forEach(ap=>{if(!ap||!ap.ssid||seen[ap.ssid])return;seen[ap.ssid]=1;const o=document.createElement('option');o.value=ap.ssid;list.appendChild(o);const opt=document.createElement('option');opt.textContent=ap.ssid+'  ('+ap.rssi+' dBm)';opt.value=ap.ssid;sel.appendChild(opt);});}\nsel.addEventListener('change',()=>{if(sel.value)ssid.value=sel.value;});\nasync function pollResult(maxTry=30){for(let i=0;i<maxTry;i++){let r=await fetch('/scan_result',{cache:'no-store'});if(!r.ok)throw new Error('result failed');let data=await r.json();if(Array.isArray(data)){return data;}if(data&&data.status!=='pending'){break;}await new Promise(res=>setTimeout(res,300));}throw new Error('timeout');}\nbtn&&btn.addEventListener('click',async()=>{btn.disabled=true;btn.textContent='Scanning...';try{const s=await fetch('/scan_start',{cache:'no-store'});if(!s.ok)throw new Error('start failed');const data=await pollResult();fillLists(data);if(data.length===0)alert('No AP found');}catch(e){alert('Scan failed');}finally{btn.disabled=false;btn.textContent='Scan Wi-Fi';}});\n</script>");
    httpd_resp_sendstr_chunk(req, "<h3>Files</h3><ul>");
    DIR *d = opendir("/sdcard");
    if (d) {
        struct dirent *de;
        while ((de = readdir(d)) != NULL) {
            if (de->d_name[0] == '.') continue;
            char esc[256]; html_escape(esc, sizeof(esc), de->d_name);
            httpd_resp_sendstr_chunk(req, "<li><a href='/download?f=");
            httpd_resp_sendstr_chunk(req, esc);
            httpd_resp_sendstr_chunk(req, "'>");
            httpd_resp_sendstr_chunk(req, esc);
            httpd_resp_sendstr_chunk(req, "</a> <a href='/delete?f=");
            httpd_resp_sendstr_chunk(req, esc);
            httpd_resp_sendstr_chunk(req, "' onclick=\"return confirm('Delete?')\">[delete]</a></li>");
        }
        closedir(d);
    } else {
        httpd_resp_sendstr_chunk(req, "<li>(SD not mounted)</li>");
    }
    httpd_resp_sendstr_chunk(req, "</ul></body></html>");
    return httpd_resp_send_chunk(req, NULL, 0);
}
*/

static esp_err_t parse_form(httpd_req_t *req, char *buf, size_t buf_sz)
{
    int total = 0;
    int cur = 0;
    int remaining = req->content_len;
    while (remaining > 0) {
        cur = httpd_req_recv(req, buf + total, MIN(remaining, (int)buf_sz - total - 1));
        if (cur <= 0) return ESP_FAIL;
        total += cur;
        remaining -= cur;
        if (total >= (int)buf_sz - 1) break;
    }
    buf[total] = 0;
    return ESP_OK;
}

static void url_decode(char *s)
{
    char *o = s;
    for (char *p = s; *p; ++p) {
        if (*p == '+') *o++ = ' ';
        else if (*p == '%' && isxdigit((unsigned char)p[1]) && isxdigit((unsigned char)p[2])) {
            char h[3] = { p[1], p[2], 0 };
            *o++ = (char)strtol(h, NULL, 16);
            p += 2;
        } else *o++ = *p;
    }
    *o = 0;
}

static char *kv_get(char *body, const char *key)
{
    size_t klen = strlen(key);
    char *p = body;
    while (p && *p) {
        char *amp = strchr(p, '&');
        if (amp) *amp = 0;
        if (!strncmp(p, key, klen) && p[klen] == '=') return p + klen + 1;
        if (!amp) break; else p = amp + 1;
    }
    return NULL;
}

/* Moved to http_server.c
static esp_err_t provision_post_handler(httpd_req_t *req)
{
    char body[512];
    if (parse_form(req, body, sizeof(body)) != ESP_OK) return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad form");
    url_decode(body);
    char *ssid = kv_get(body, "ssid");
    char *pass = kv_get(body, "pass");
    char *api  = kv_get(body, "api");
    if (!ssid) ssid = "";
    if (!pass) pass = "";
    if (!api) api = "";

    // 只有当SSID不为空时才更新WiFi配置
    if (strlen(ssid) > 0) {
        // 校验密码：若非空且小于8位，则返回错误并保持在配网模式
        if (strlen(pass) > 0 && strlen(pass) < 8) {
            ESP_LOGW(TAG, "Provision rejected: password too short (%d)", (int)strlen(pass));
            httpd_resp_set_type(req, "text/html; charset=utf-8");
            httpd_resp_set_status(req, "400 Bad Request");
            return httpd_resp_sendstr(req, "<html><body><p>Password must be at least 8 characters for WPA/WPA2 networks.</p><p><a href='/'>&larr; Back</a></p></body></html>");
        }
        nvs_handle_t n;
        if (nvs_open(NVS_NS_WIFI, NVS_READWRITE, &n) == ESP_OK) {
            nvs_set_str(n, NVS_KEY_SSID, ssid);
            nvs_set_str(n, NVS_KEY_PASS, pass);
            nvs_commit(n);
            nvs_close(n);
            ESP_LOGI(TAG, "WiFi credentials updated: SSID=%s", ssid);
        }

        // 切换为正常工作模式：仅STA，不再广播配网SSID
        // 注意：在HTTP响应后客户端可能断开（AP被关闭属预期行为）
        wifi_config_t wifi_sta = {0};
        strncpy((char *)wifi_sta.sta.ssid, ssid, sizeof(wifi_sta.sta.ssid));
        strncpy((char *)wifi_sta.sta.password, pass, sizeof(wifi_sta.sta.password));
        // 根据密码是否为空设置阈值：空密码=开放网络；否则=WPA2
        wifi_sta.sta.threshold.authmode = (strlen(pass) == 0) ? WIFI_AUTH_OPEN : WIFI_AUTH_WPA2_PSK;
        wifi_sta.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
        wifi_sta.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
        wifi_sta.sta.listen_interval = 3;
        
        s_provisioning_mode = false;
        esp_wifi_set_mode(WIFI_MODE_STA);
        esp_wifi_set_config(WIFI_IF_STA, &wifi_sta);
        esp_wifi_connect();
    }
    
    // 总是更新API Key（即使是空值）
    nvs_handle_t n;
    if (nvs_open(NVS_NS_APP, NVS_READWRITE, &n) == ESP_OK) {
        nvs_set_str(n, NVS_KEY_API, api);
        nvs_commit(n);
        nvs_close(n);
        ESP_LOGI(TAG, "API Key %s", strlen(api) > 0 ? "updated" : "cleared");
    }
    
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "/");
    return httpd_resp_send(req, NULL, 0);
}
*/

static void send_scan_json(httpd_req_t *req, const wifi_ap_record_t *recs, uint16_t ap_num)
{
    httpd_resp_set_type(req, "application/json; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_sendstr_chunk(req, "[");
    for (uint16_t i = 0; i < ap_num; ++i) {
        char ssid_esc[66] = {0};
        size_t di = 0;
        for (int j = 0; j < 32 && recs[i].ssid[j]; ++j) {
            unsigned char c = recs[i].ssid[j];
            if (c == '"' || c == '\\') {
                if (di + 2 >= sizeof(ssid_esc)) break;
                ssid_esc[di++] = '\\'; ssid_esc[di++] = c;
            } else if (c >= 0x20 && c < 0x7F) {
                if (di + 1 >= sizeof(ssid_esc)) break;
                ssid_esc[di++] = c;
            } else {
                if (di + 1 >= sizeof(ssid_esc)) break;
                ssid_esc[di++] = '?';
            }
        }
        char item[160];
        int n = snprintf(item, sizeof(item),
                         "%s{\"ssid\":\"%s\",\"rssi\":%d,\"auth\":%d}",
                         (i ? "," : ""), ssid_esc, recs[i].rssi, recs[i].authmode);
        if (n > 0 && n < (int)sizeof(item)) httpd_resp_sendstr_chunk(req, item);
    }
    httpd_resp_sendstr_chunk(req, "]");
    httpd_resp_send_chunk(req, NULL, 0);
}

/* Moved to http_server.c
static esp_err_t scan_start_get_handler(httpd_req_t *req)
{
    if (!s_scan_mutex) s_scan_mutex = xSemaphoreCreateMutex();
    if (s_scan_mutex) xSemaphoreTake(s_scan_mutex, portMAX_DELAY);
    // cancel prior
    esp_wifi_scan_stop();
    if (s_scan_results) { free(s_scan_results); s_scan_results = NULL; s_scan_count = 0; }
    wifi_scan_config_t scan_cfg = {
        .ssid = 0,
        .bssid = 0,
        .channel = 0,
        .show_hidden = false,
        .scan_type = WIFI_SCAN_TYPE_ACTIVE,
        .scan_time = { .active = { .min = 30, .max = 60 } },
    };
    esp_err_t err = esp_wifi_scan_start(&scan_cfg, false);
    if (err == ESP_OK) {
        s_scan_busy = true;
        ESP_LOGI(TAG, "HTTP /scan_start: started");
        if (s_scan_mutex) xSemaphoreGive(s_scan_mutex);
        httpd_resp_set_type(req, "application/json");
        return httpd_resp_sendstr(req, "{\"status\":\"started\"}");
    } else {
        if (s_scan_mutex) xSemaphoreGive(s_scan_mutex);
        ESP_LOGE(TAG, "HTTP /scan_start error: %s", esp_err_to_name(err));
        char msg[96];
        snprintf(msg, sizeof msg, "{\"status\":\"error\",\"code\":%d}", err);
        httpd_resp_set_type(req, "application/json");
        return httpd_resp_sendstr(req, msg);
    }
}
*/

/* Moved to http_server.c
static esp_err_t scan_result_get_handler(httpd_req_t *req)
{
    if (!s_scan_mutex) s_scan_mutex = xSemaphoreCreateMutex();
    if (s_scan_mutex) xSemaphoreTake(s_scan_mutex, portMAX_DELAY);
    bool busy = s_scan_busy;
    wifi_ap_record_t *recs = s_scan_results;
    uint16_t num = s_scan_count;
    if (s_scan_mutex) xSemaphoreGive(s_scan_mutex);
    if (busy) {
        httpd_resp_set_type(req, "application/json");
        return httpd_resp_sendstr(req, "{\"status\":\"pending\"}");
    }
    // send cached results as JSON array
    if (recs && num > 0) {
        send_scan_json(req, recs, num);
        return ESP_OK;
    }
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "[]");
}
*/

static esp_err_t download_get_handler(httpd_req_t *req)
{
    char path[256];
    char f[192];
    if (httpd_req_get_url_query_str(req, path, sizeof(path)) != ESP_OK) return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "no query");
    if (httpd_query_key_value(path, "f", f, sizeof(f)) != ESP_OK) return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "no f");
    char full[256]; snprintf(full, sizeof(full), "/sdcard/%s", f);
    FILE *fp = fopen(full, "rb");
    if (!fp) return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "not found");
    httpd_resp_set_type(req, "application/octet-stream");
    httpd_resp_set_hdr(req, "Content-Disposition", "attachment");
    char buf[1024]; size_t n;
    while ((n = fread(buf, 1, sizeof(buf), fp)) > 0) {
        if (httpd_resp_send_chunk(req, buf, n) != ESP_OK) break;
    }
    fclose(fp);
    return httpd_resp_send_chunk(req, NULL, 0);
}

static esp_err_t delete_get_handler(httpd_req_t *req)
{
    char path[256];
    char f[192];
    if (httpd_req_get_url_query_str(req, path, sizeof(path)) != ESP_OK) return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "no query");
    if (httpd_query_key_value(path, "f", f, sizeof(f)) != ESP_OK) return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "no f");
    char full[256]; snprintf(full, sizeof(full), "/sdcard/%s", f);
    unlink(full);
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "/");
    return httpd_resp_send(req, NULL, 0);
}

/* HTTP server moved to http_server.c */

/* Wi-Fi events moved to wifi_mgr.c */

/* Wi-Fi init moved to wifi_mgr.c */

// --- WebSocket ASR ---

static void ws_send_run_task(void)
{
    if (!s_ws || !s_ws_connected) return;
    memset(s_ws_task_id, 0, sizeof(s_ws_task_id));
    gen_uuid_like(s_ws_task_id, sizeof(s_ws_task_id));
    char json[512];
    int n = snprintf(json, sizeof(json),
        "{\"header\":{\"action\":\"run-task\",\"task_id\":\"%s\",\"streaming\":\"duplex\"},"
        "\"payload\":{\"task_group\":\"audio\",\"task\":\"asr\",\"function\":\"recognition\",\"model\":\"paraformer-realtime-v2\","
        "\"parameters\":{\"format\":\"pcm\",\"sample_rate\":%d,\"inverse_text_normalization_enabled\":true,\"punctuation_prediction_enabled\":true},\"input\":{}}}",
        s_ws_task_id, MIC_SAMPLE_RATE);
    if (n > 0 && n < (int)sizeof(json)) {
        esp_websocket_client_send_text(s_ws, json, n, portMAX_DELAY);
        ESP_LOGI(TAG, "WS run-task sent");
    }
}

static void ws_send_finish_task(void)
{
    if (!s_ws || !s_ws_connected || !s_ws_task_started) return;
    char json[256];
    int n = snprintf(json, sizeof(json),
                     "{\"header\":{\"action\":\"finish-task\",\"task_id\":\"%s\",\"streaming\":\"duplex\"},\"payload\":{\"input\":{}}}",
                     s_ws_task_id);
    if (n > 0 && n < (int)sizeof(json)) {
        esp_websocket_client_send_text(s_ws, json, n, portMAX_DELAY);
        ESP_LOGI(TAG, "WS finish-task sent");
    }
}

static void ensure_transcript_file_open(void)
{
    if (s_transcript_fp || s_current_rec_base[0] == 0) return;
    char tpath[160];
    snprintf(tpath, sizeof(tpath), "%s.txt", s_current_rec_base);
    s_transcript_fp = fopen(tpath, "a");
}

static const char *bytes_search(const char *hay, size_t hay_len, const char *needle, size_t nee_len)
{
    if (nee_len == 0 || hay_len < nee_len) return NULL;
    const char *end = hay + hay_len - nee_len + 1;
    for (const char *p = hay; p < end; ++p) {
        if (p[0] == needle[0] && memcmp(p, needle, nee_len) == 0) return p;
    }
    return NULL;
}

static const char *find_json_field(const char *data, int len, const char *key)
{
    // naive search: "key":"value"
    static char pat[64];
    int k = snprintf(pat, sizeof(pat), "\"%s\":\"", key);
    if (k <= 0) return NULL;
    const char *p = bytes_search(data, (size_t)len, pat, (size_t)k);
    if (!p) return NULL;
    p += k;
    return p; // points at value start
}

static bool json_flag_true(const char *data, int len, const char *key)
{
    static char pat[64];
    int k = snprintf(pat, sizeof(pat), "\"%s\":true", key);
    if (k <= 0) return false;
    return bytes_search(data, (size_t)len, pat, (size_t)k) != NULL;
}

static void extract_string(const char *start, const char *end, char *out, size_t out_sz)
{
    size_t i = 0;
    for (const char *p = start; p < end && i + 1 < out_sz; ++p) {
        if (*p == '\\' && p + 1 < end) {
            p++;
            char c = *p;
            if (c == 'n') out[i++]='\n';
            else if (c == 'r') out[i++]='\r';
            else if (c == 't') out[i++]='\t';
            else out[i++]=c;
        } else if (*p == '"') {
            break;
        } else {
            out[i++] = *p;
        }
    }
    out[i]=0;
}

static void ws_on_text_message(const char *data, int len)
{
    // Event type
    const char *evp = find_json_field(data, len, "event");
    if (evp) {
        const char *ev_end = (const char *)memchr(evp, '"', (data + len) - evp);
        if (ev_end) {
            char ev[32]; extract_string(evp, ev_end, ev, sizeof ev);
            if (strcmp(ev, "task-started") == 0) {
                s_ws_task_started = true;
                ESP_LOGI(TAG, "WS task-started");
                return;
            }
            if (strcmp(ev, "task-finished") == 0) {
                ESP_LOGI(TAG, "WS task-finished");
                s_ws_task_started = false;
                return;
            }
        }
    }
    // For result-generated: extract sentence.text and sentence_end
    const char *tp = find_json_field(data, len, "text");
    if (tp) {
        const char *tend = (const char *)memchr(tp, '"', (data + len) - tp);
        if (tend) {
            char line[512]; extract_string(tp, tend, line, sizeof line);
            show_text(line);
            ESP_LOGI(TAG, "ASR: %s", line);
            bool sent_end = json_flag_true(data, len, "sentence_end");
            if (sent_end) {
                ensure_transcript_file_open();
                if (s_transcript_fp) { fprintf(s_transcript_fp, "%s\n", line); fflush(s_transcript_fp); }
            }
        }
    }
}

static void ws_event_handler(void *arg, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_websocket_event_data_t *e = (esp_websocket_event_data_t *)event_data;
    switch (event_id) {
        case WEBSOCKET_EVENT_CONNECTED:
            s_ws_connected = true;
            ESP_LOGI(TAG, "WS connected");
            ws_send_run_task();
            break;
        case WEBSOCKET_EVENT_DISCONNECTED:
            s_ws_connected = false;
            s_ws_task_started = false;
            ESP_LOGW(TAG, "WS disconnected");
            break;
        case WEBSOCKET_EVENT_DATA:
            if (e->op_code == 0x1) { // text
                ws_on_text_message((const char *)e->data_ptr, e->data_len);
            }
            break;
        case WEBSOCKET_EVENT_ERROR:
            ESP_LOGE(TAG, "WS error");
            break;
    }
}

static void ws_task(void *arg)
{
    // Read API key from NVS
    nvs_handle_t n;
    char api_key[160] = {0};
    size_t len = sizeof(api_key);
    if (nvs_open(NVS_NS_APP, NVS_READONLY, &n) == ESP_OK) {
        if (nvs_get_str(n, NVS_KEY_API, api_key, &len) != ESP_OK) api_key[0] = 0;
        nvs_close(n);
    }
    if (api_key[0] == 0) {
        ESP_LOGW(TAG, "No API key in NVS; ASR disabled");
        vTaskDelete(NULL);
        return;
    }

    char auth_hdr[400];
    // Build headers: Authorization + optional Origin
    if (strlen(CONFIG_GEEK_WS_ORIGIN) > 0)
        snprintf(auth_hdr, sizeof(auth_hdr),
                 "Authorization: Bearer %s\r\nOrigin: %s",
                 api_key, CONFIG_GEEK_WS_ORIGIN);
    else
        snprintf(auth_hdr, sizeof(auth_hdr),
                 "Authorization: Bearer %s",
                 api_key);

    esp_websocket_client_config_t cfg = {
        .uri = CONFIG_GEEK_WS_URI,
        .headers = auth_hdr,
        .subprotocol = (strlen(CONFIG_GEEK_WS_SUBPROTOCOL) ? CONFIG_GEEK_WS_SUBPROTOCOL : NULL),
        .network_timeout_ms = 10000,
        .reconnect_timeout_ms = 5000,
        // Use IDF certificate bundle so we don't need to preload global CA store
        .crt_bundle_attach = esp_crt_bundle_attach,
    };
    s_ws = esp_websocket_client_init(&cfg);
    ESP_LOGI(TAG, "WS init: uri=%s, subprotocol=%s", CONFIG_GEEK_WS_URI,
             strlen(CONFIG_GEEK_WS_SUBPROTOCOL) ? CONFIG_GEEK_WS_SUBPROTOCOL : "<none>");
    esp_websocket_register_events(s_ws, WEBSOCKET_EVENT_ANY, ws_event_handler, NULL);
    esp_websocket_client_start(s_ws);

    // Simple loop to keep task alive; send finish-task on deletion
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void ws_start_if_ready(void)
{
    // Start once
    static bool started = false;
    if (!started) {
        started = true;
        xTaskCreatePinnedToCore(ws_task, "ws_asr", 6144, NULL, 5, NULL, tskNO_AFFINITY);
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Geek Recorder boot");
    
    // 优先初始化BOOT按键处理逻辑，确保在系统出现问题时也能进入恢复模式
    ESP_LOGI(TAG, "Creating BOOT key task early");
    BaseType_t result = xTaskCreatePinnedToCore(boot_key_task, "boot_key", 4096, NULL, 5, NULL, tskNO_AFFINITY);
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create BOOT key task, error code: %d", result);
    } else {
        ESP_LOGI(TAG, "BOOT key task created successfully");
        // 给BOOT按键任务一些时间来初始化
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    ESP_ERROR_CHECK(nvs_flash_init());

    // Init LCD early for status (no 'Booting' banner after startup)
    lcd_init();

    // Mount TF
    bool sd_ok = (sdcard_mount() == ESP_OK);
    if (!sd_ok) {
        ESP_LOGW(TAG, "SD card not mounted");
    }

    // Init Wi-Fi (provisioning AP or STA)
    wifi_mgr_init();

    // Start status bar updater
    xTaskCreatePinnedToCore(status_task, "ui_status", 4096, NULL, 4, NULL, tskNO_AFFINITY);
    
    // Initialize SNTP for time synchronization after Wi-Fi is initialized
    // 添加延迟确保网络栈完全初始化
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_init();
    
    // Wait for time to be set (with timeout)
    time_t now = 0;
    struct tm timeinfo = { 0 };
    int retry = 0;
    const int retry_count = 10;
    while (esp_sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < retry_count) {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
    time(&now);
    localtime_r(&now, &timeinfo);
    ESP_LOGI(TAG, "Current time: %04d-%02d-%02d %02d:%02d:%02d", 
             timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
             timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);

    // Init mic and start capture task (writes WAV to /sdcard) only if SD card is mounted
    if (sd_ok && mic_init() == ESP_OK) {
        xTaskCreatePinnedToCore(mic_capture_task, "mic_capture", 4096, NULL, 5, NULL, tskNO_AFFINITY);
    } else if (!sd_ok) {
        ESP_LOGW(TAG, "Skipping mic capture task: SD card not mounted");
    }

    // HTTP server for provisioning + file mgmt
    http_server_start(NULL);

    // Start ASR WS client (if API key exists)
    ws_start_if_ready();

#ifdef CONFIG_GEEK_USE_LVGL
    // Register LVGL FS driver for SD and try to load font from S:/fonts/chs_16.bin
    if (s_lvgl_ok && sd_ok) {
        lvgl_fs_sd_register();
        lvgl_port_lock();
#if LV_USE_FONT_LOADER
        // Requires LV_USE_FONT_LOADER=1 (enabled in lv_conf.h)
        s_font_chs = lv_font_load("S:/fonts/chs_16.bin");
        if (s_font_chs) {
            lv_obj_set_style_text_font(s_label, s_font_chs, LV_PART_MAIN);
            ESP_LOGI(TAG, "Loaded LVGL font from SD");
        } else {
            ESP_LOGW(TAG, "Failed to load font from SD; using default");
        }
#else
        ESP_LOGW(TAG, "LV_USE_FONT_LOADER=0; cannot load font from SD");
#endif
        lvgl_port_unlock();
    }
#endif

    // Placeholder: WebSocket client for ASR integration will be implemented later.
    ESP_LOGI(TAG, "System initialized");
}
