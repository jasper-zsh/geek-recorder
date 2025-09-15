#include "lvgl.h"
#include "lvgl_port.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_heap_caps.h"

static const char *TAG = "lvgl_port";
static SemaphoreHandle_t s_mutex;
static esp_lcd_panel_handle_t s_panel;
static int s_hres, s_vres;
static lv_disp_draw_buf_t s_draw_buf;
static lv_color_t *s_buf1 = NULL;
static lv_color_t *s_buf2 = NULL;
static lv_disp_drv_t s_disp_drv;

static void lvgl_tick_cb(void *arg)
{
    lv_tick_inc(1);
}

static void flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_p)
{
    int x1 = area->x1;
    int y1 = area->y1;
    int x2 = area->x2;
    int y2 = area->y2;
    // LVGL uses inclusive x2/y2; esp_lcd expects exclusive end
    esp_lcd_panel_draw_bitmap(s_panel, x1, y1, x2 + 1, y2 + 1, color_p);
    lv_disp_flush_ready(drv);
}

static void gui_task(void *arg)
{
    while (1) {
        lvgl_port_lock();
        lv_timer_handler();
        lvgl_port_unlock();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

bool lvgl_port_init(esp_lcd_panel_handle_t panel, int hor_res, int ver_res, int task_stack)
{
    s_panel = panel; s_hres = hor_res; s_vres = ver_res;
    s_mutex = xSemaphoreCreateRecursiveMutex();
    if (!s_mutex) return false;

    lv_init();

    // Double buffer: a few lines tall to save RAM
    int buf_lines = 20; // smaller chunk to reduce flush time
    int px_count = hor_res * buf_lines;
    s_buf1 = heap_caps_malloc(px_count * sizeof(lv_color_t), MALLOC_CAP_DMA);
    s_buf2 = heap_caps_malloc(px_count * sizeof(lv_color_t), MALLOC_CAP_DMA);
    if (!s_buf1 || !s_buf2) {
        ESP_LOGE(TAG, "LVGL draw buffer alloc failed");
        return false;
    }
    lv_disp_draw_buf_init(&s_draw_buf, s_buf1, s_buf2, px_count);

    lv_disp_drv_init(&s_disp_drv);
    s_disp_drv.hor_res = hor_res;
    s_disp_drv.ver_res = ver_res;
    s_disp_drv.flush_cb = flush_cb;
    s_disp_drv.draw_buf = &s_draw_buf;
    lv_disp_drv_register(&s_disp_drv);

    // 1ms tick
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &lvgl_tick_cb,
        .name = "lvgl_tick"
    };
    esp_timer_handle_t timer;
    esp_timer_create(&periodic_timer_args, &timer);
    esp_timer_start_periodic(timer, 1000);

    xTaskCreatePinnedToCore(gui_task, "lvgl", task_stack, NULL, tskIDLE_PRIORITY + 1, NULL, tskNO_AFFINITY);
    return true;
}

void lvgl_port_lock(void)
{
    if (s_mutex) xSemaphoreTakeRecursive(s_mutex, portMAX_DELAY);
}

void lvgl_port_unlock(void)
{
    if (s_mutex) xSemaphoreGiveRecursive(s_mutex);
}
