#pragma once
#include "esp_lcd_panel_ops.h"
#include <stdbool.h>

bool lvgl_port_init(esp_lcd_panel_handle_t panel, int hor_res, int ver_res, int task_stack);
void lvgl_port_lock(void);
void lvgl_port_unlock(void);
void lvgl_fs_sd_register(void);
