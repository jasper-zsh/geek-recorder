#include "esp_log.h"
#include <errno.h>
#include "esp_vfs_fat.h"
#include "driver/sdmmc_host.h"
#include "driver/sdmmc_defs.h"
#include "sdmmc_cmd.h"

#include "sdcard.h"

static const char *TAG = "sdcard";

// SDMMC config from Kconfig
#define SD_CLK_GPIO       CONFIG_GEEK_SDMMC_CLK
#define SD_CMD_GPIO       CONFIG_GEEK_SDMMC_CMD
#define SD_D0_GPIO        CONFIG_GEEK_SDMMC_D0
#define SD_D1_GPIO        CONFIG_GEEK_SDMMC_D1
#define SD_D2_GPIO        CONFIG_GEEK_SDMMC_D2
#define SD_D3_GPIO        CONFIG_GEEK_SDMMC_D3

static sdmmc_card_t *s_card = NULL;

bool sdcard_mount(void)
{
    ESP_LOGI(TAG, "Mount SD card (SDMMC 4-bit)");
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.max_freq_khz = SDMMC_FREQ_DEFAULT;
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.clk = SD_CLK_GPIO;
    slot_config.cmd = SD_CMD_GPIO;
    slot_config.d0  = SD_D0_GPIO;
    slot_config.d1  = SD_D1_GPIO;
    slot_config.d2  = SD_D2_GPIO;
    slot_config.d3  = SD_D3_GPIO;
    slot_config.width = 4;

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = true,
        .max_files = 16, // allow more simultaneous opens across UI/HTTP/mic
        .allocation_unit_size = 16 * 1024,
    };

    esp_err_t ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &s_card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount SD card: %s", esp_err_to_name(ret));
        return false;
    }
    sdmmc_card_print_info(stdout, s_card);

    // Quick write test to catch read-only mounts early
    // Use 8.3-compatible name to work even if LFN is off
    FILE *tf = fopen("/sdcard/writetest.tmp", "wb");
    if (!tf) {
        ESP_LOGE(TAG, "Mounted but not writable (errno=%d)", errno);
        return false;
    }
    fwrite("ok", 1, 2, tf);
    fclose(tf);
    remove("/sdcard/writetest.tmp");
    return true;
}
