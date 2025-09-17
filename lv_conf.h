#ifndef LV_CONF_H
#define LV_CONF_H

#define LV_CONF_INCLUDE_SIMPLE 1

// Enable runtime font loader for loading .lvgl font binaries from filesystem
#define LV_USE_FONT_LOADER 1

// Enable support for compressed runtime fonts (e.g., LVGL binary with LZ4)
#define LV_USE_FONT_COMPRESSED 1

// Keep other options default; color depth and byte order are configured via sdkconfig Kconfig

#endif // LV_CONF_H
