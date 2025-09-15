#include <stdio.h>
#include <string.h>
#include "lvgl.h"
#include "esp_log.h"

static const char *TAG = "lvgl_fs_sd";

typedef struct {
    FILE *fp;
} file_t;

static void * sd_open(lv_fs_drv_t * drv, const char * path, lv_fs_mode_t mode)
{
    LV_UNUSED(drv);
    const char *flags = (mode == LV_FS_MODE_WR) ? "wb" : (mode == LV_FS_MODE_RD) ? "rb" : "rb+";
    char full[256];
    // Map S:/xxx -> /sdcard/xxx
    snprintf(full, sizeof(full), "/sdcard/%s", path);
    FILE *fp = fopen(full, flags);
    if (!fp) return NULL;
    file_t *f = (file_t *)lv_mem_alloc(sizeof(file_t));
    if (!f) { fclose(fp); return NULL; }
    f->fp = fp;
    return f;
}

static lv_fs_res_t sd_close(lv_fs_drv_t * drv, void * file_p)
{
    LV_UNUSED(drv);
    file_t *f = (file_t *)file_p;
    if (!f) return LV_FS_RES_OK;
    fclose(f->fp);
    lv_mem_free(f);
    return LV_FS_RES_OK;
}

static lv_fs_res_t sd_read(lv_fs_drv_t * drv, void * file_p, void * buf, uint32_t btr, uint32_t * br)
{
    LV_UNUSED(drv);
    file_t *f = (file_t *)file_p;
    size_t n = fread(buf, 1, btr, f->fp);
    if (br) *br = (uint32_t)n;
    if (n < btr && ferror(f->fp)) return LV_FS_RES_FS_ERR;
    return LV_FS_RES_OK;
}

static lv_fs_res_t sd_seek(lv_fs_drv_t * drv, void * file_p, uint32_t pos, lv_fs_whence_t whence)
{
    LV_UNUSED(drv);
    file_t *f = (file_t *)file_p;
    int origin = (whence == LV_FS_SEEK_SET) ? SEEK_SET : (whence == LV_FS_SEEK_CUR) ? SEEK_CUR : SEEK_END;
    if (fseek(f->fp, (long)pos, origin) != 0) return LV_FS_RES_FS_ERR;
    return LV_FS_RES_OK;
}

static lv_fs_res_t sd_tell(lv_fs_drv_t * drv, void * file_p, uint32_t * pos_p)
{
    LV_UNUSED(drv);
    file_t *f = (file_t *)file_p;
    long p = ftell(f->fp);
    if (p < 0) return LV_FS_RES_FS_ERR;
    if (pos_p) *pos_p = (uint32_t)p;
    return LV_FS_RES_OK;
}

void lvgl_fs_sd_register(void)
{
    lv_fs_drv_t drv;
    lv_fs_drv_init(&drv);
    drv.letter = 'S';
    drv.open_cb = sd_open;
    drv.close_cb = sd_close;
    drv.read_cb = sd_read;
    drv.seek_cb = sd_seek;
    drv.tell_cb = sd_tell;
    lv_fs_drv_register(&drv);
    ESP_LOGI(TAG, "LVGL FS driver 'S:' -> /sdcard registered");
}

