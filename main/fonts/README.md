# LVGL Chinese Font

To display Chinese characters in LVGL:

1. Use the LVGL font converter (web or CLI) to generate a font C file from a CJK TTF such as Noto Sans CJK SC.
   - Range: Basic Latin + commonly used CJK (or All if flash allows)
   - Size: 16 px (recommended for 135x240)
   - Bpp: 4 or 2 (trade clarity vs flash)
   - Enable Subpixel off; Enable compression on
   - Font name (symbol): `lv_font_chs_16`

2. Put the generated `.c` file here and declare in a header, e.g.:

```c
#include "lvgl.h"
LV_FONT_DECLARE(lv_font_chs_16);
```

3. In `app_main.c`, LVGL will use `lv_font_chs_16` if present once you set it for a style/theme; otherwise built‑in Latin font is used and Chinese will not render.

Notes:
- Ensure `CONFIG_LV_COLOR_DEPTH_16` and `CONFIG_LV_COLOR_16_SWAP` are enabled (sdkconfig.defaults has presets).
- For larger fonts or full CJK coverage, consider storing the font in external flash (W25Q128) and enabling link‑time placement to that region.

