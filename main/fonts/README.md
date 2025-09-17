# LVGL Chinese Font

LVGL now loads the Chinese font at runtime from the SD card. To prepare and install the font:

1. Use the LVGL font converter (web or CLI) to generate a **binary** (`.bin`) font from a CJK TTF such as Noto Sans CJK SC.
   - Range: Basic Latin + the Chinese characters you need (or full range if storage allows)
   - Size: 16 px works well for the 135×240 LCD
   - Bpp: 4 (balanced quality vs. file size)
   - Compression: enabled
   - Symbol/name: `chs_16`

2. Upload the resulting `chs_16.bin` through the device web UI (Font Library section). The firmware stores it as `/fonts/chs_16.bin` on the SD card and reloads it immediately.

3. Reopen the web page or check the LCD to verify that Chinese glyphs render correctly. If loading fails, the UI falls back to the built-in Latin font.

Notes:
- Ensure `LV_USE_FONT_LOADER` remains enabled in `lv_conf.h` (already set).
- The SD card must remain inserted; removing it reverts the UI to the default Latin font.
