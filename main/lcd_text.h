#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_lcd_panel_ops.h"

typedef struct {
    int cols;
    int rows;
    uint16_t fg;
    uint16_t bg;
} lcd_text_cfg_t;

void lcd_text_init(esp_lcd_panel_handle_t panel, int lcd_w, int lcd_h, const lcd_text_cfg_t *cfg);
void lcd_text_clear(void);
void lcd_text_println(const char *s);
void lcd_text_render(void);
void lcd_text_set_colors(uint16_t fg, uint16_t bg);

