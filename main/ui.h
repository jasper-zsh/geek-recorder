#pragma once

#include <stdbool.h>

// Initialize LCD/LVGL UI. Safe to call once at boot.
void ui_init(void);

// Append a text line to the log area (if available).
void ui_show_text(const char *text);

// Transcript helpers for ASR: partial replaces the current tail, sentence appends with a newline.
void ui_transcript_set_partial(const char *text);
void ui_transcript_add_sentence(const char *text);

// Update top status line (Wi‑Fi/IP/REC/ASR/time).
void ui_update_status_line(void);

// Start a periodic task that updates the status line once per second.
void ui_start_status_task(void);

// Optional post‑SD initialization: register LVGL FS and try loading fonts.
void ui_post_sd_init(void);

// Reload LVGL font from SD card; returns true if loaded successfully.
bool ui_reload_font_from_sd(void);

// Called when recording starts to record the start time
void ui_record_started(void);
