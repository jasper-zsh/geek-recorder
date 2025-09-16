#pragma once

#include <stdbool.h>

// Initialize LCD/LVGL UI. Safe to call once at boot.
void ui_init(void);

// Append a text line to the log area (if available).
void ui_show_text(const char *text);

// Update top status line (Wi‑Fi/IP/REC/ASR/time).
void ui_update_status_line(void);

// Start a periodic task that updates the status line once per second.
void ui_start_status_task(void);

// Optional post‑SD initialization: register LVGL FS and try loading fonts.
void ui_post_sd_init(void);

