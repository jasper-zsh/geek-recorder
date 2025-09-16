#pragma once
#include <stdbool.h>

// Mount SD card to /sdcard using SDMMC 4-bit mode.
// Returns true on success.
bool sdcard_mount(void);

