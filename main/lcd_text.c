#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include "lcd_text.h"
#include "font8x8_basic.h"
#include "esp_heap_caps.h"
#include "esp_log.h"

static const char *TAG = "lcd_text";

static esp_lcd_panel_handle_t s_panel;
static int s_lcd_w, s_lcd_h;
static int s_cols, s_rows;
static uint16_t s_fg = 0xFFFF; // white
static uint16_t s_bg = 0x0000; // black
static char *s_buf = NULL;      // text buffer rows*cols
static int s_cursor_row = 0;    // next write row (scroll)
static uint16_t *s_frame = NULL; // RGB565 framebuffer for text area (rows*8 height)

static inline uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b) {
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

void lcd_text_set_colors(uint16_t fg, uint16_t bg) {
    s_fg = fg; s_bg = bg;
}

void lcd_text_init(esp_lcd_panel_handle_t panel, int lcd_w, int lcd_h, const lcd_text_cfg_t *cfg)
{
    s_panel = panel; s_lcd_w = lcd_w; s_lcd_h = lcd_h;
    s_cols = cfg && cfg->cols ? cfg->cols : (lcd_w / 8);
    s_rows = cfg && cfg->rows ? cfg->rows : (lcd_h / 8);
    s_fg = cfg ? cfg->fg : 0xFFFF;
    s_bg = cfg ? cfg->bg : 0x0000;
    size_t sz = (size_t)s_cols * s_rows;
    s_buf = (char *)heap_caps_calloc(sz, 1, MALLOC_CAP_DEFAULT);
    s_frame = (uint16_t *)heap_caps_malloc((size_t)s_cols * 8 * s_rows * 8 * sizeof(uint16_t), MALLOC_CAP_DMA);
    if (!s_buf || !s_frame) {
        ESP_LOGE(TAG, "alloc failed cols=%d rows=%d", s_cols, s_rows);
    }
    lcd_text_clear();
}

void lcd_text_clear(void)
{
    if (!s_buf) return;
    memset(s_buf, ' ', (size_t)s_cols * s_rows);
    s_cursor_row = 0;
    lcd_text_render();
}

static void draw_glyph_to_fb(int gx, int gy, unsigned char ch)
{
    const unsigned char *glyph = NULL;
    if (ch >= 0x20 && ch <= 0x7F) glyph = font8x8_basic[ch - 0x20];
    // Non-ASCII placeholder: 0xFE pattern box
    unsigned char placeholder[8] = {0x7E,0x42,0x5A,0x5A,0x5A,0x42,0x7E,0x00};
    if (!glyph) glyph = placeholder;
    int px = gx * 8;
    int py = gy * 8;
    for (int y = 0; y < 8; ++y) {
        unsigned char row = glyph[y];
        uint16_t *dst = s_frame + (py + y) * (s_cols * 8) + px;
        for (int x = 0; x < 8; ++x) {
            bool on = (row >> (7 - x)) & 0x01;
            dst[x] = on ? s_fg : s_bg;
        }
    }
}

void lcd_text_render(void)
{
    if (!s_buf || !s_frame) return;
    int fb_w = s_cols * 8;
    int fb_h = s_rows * 8;
    // paint bg
    for (int i = 0; i < fb_w * fb_h; ++i) s_frame[i] = s_bg;
    // draw glyphs
    for (int r = 0; r < s_rows; ++r) {
        for (int c = 0; c < s_cols; ++c) {
            unsigned char ch = (unsigned char)s_buf[r * s_cols + c];
            draw_glyph_to_fb(c, r, ch);
        }
    }
    // center vertically if fb_h < s_lcd_h
    int y0 = 0;
    if (fb_h < s_lcd_h) y0 = (s_lcd_h - fb_h) / 2;
    // draw
    esp_lcd_panel_draw_bitmap(s_panel, 0, y0, fb_w, y0 + fb_h, s_frame);
}

static void scroll_up(int lines)
{
    if (!s_buf) return;
    if (lines <= 0 || lines > s_rows) return;
    size_t line_bytes = (size_t)s_cols;
    memmove(s_buf, s_buf + lines * s_cols, (size_t)(s_rows - lines) * s_cols);
    memset(s_buf + (size_t)(s_rows - lines) * s_cols, ' ', (size_t)lines * s_cols);
}

static void putc_at(int row, int col, char ch)
{
    if (row < 0 || row >= s_rows || col < 0 || col >= s_cols) return;
    s_buf[row * s_cols + col] = ch;
}

void lcd_text_println(const char *s)
{
    if (!s_buf) return;
    // write on next row; wrap long lines
    int col = 0;
    const unsigned char *p = (const unsigned char *)s;
    while (*p) {
        unsigned char ch = *p++;
        if (ch == '\n') break;
        if (ch < 0x20) ch = ' ';
        // treat UTF-8 multi-byte as placeholder
        if (ch >= 0x80) { ch = '?'; }
        putc_at(s_cursor_row, col, (char)ch);
        if (++col >= s_cols) {
            s_cursor_row++;
            col = 0;
            if (s_cursor_row >= s_rows) { scroll_up(1); s_cursor_row = s_rows - 1; }
        }
    }
    // advance to next line
    s_cursor_row++;
    if (s_cursor_row >= s_rows) { scroll_up(1); s_cursor_row = s_rows - 1; }
    lcd_text_render();
}

