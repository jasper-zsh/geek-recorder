#include <stdio.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_heap_caps.h"
#include "driver/gpio.h"
#include "driver/i2s_std.h"
#include "driver/i2s_pdm.h"

#include "mic.h"
#include "asr_ws.h"
#include <time.h>
#include <errno.h>

static const char *TAG = "mic";

// MIC config from Kconfig
#define MIC_GPIO_CLK      CONFIG_GEEK_MIC_PDM_CLK
#define MIC_GPIO_DATA     CONFIG_GEEK_MIC_PDM_DATA
#define MIC_GPIO_SEL      CONFIG_GEEK_MIC_SEL_GPIO
#define MIC_SAMPLE_RATE   CONFIG_GEEK_MIC_SAMPLE_RATE

static i2s_chan_handle_t s_pdm_rx_chan = NULL;
static volatile bool s_record_request = false;
static volatile bool s_recording_on = false;
static FILE *s_wav_fp = NULL;

#define MIC_AGC_TARGET_RMS        4000.0f
#define MIC_AGC_MAX_GAIN          18.0f
#define MIC_AGC_MIN_GAIN          0.35f
#define MIC_AGC_ATTACK_ALPHA      0.35f
#define MIC_AGC_RELEASE_ALPHA     0.010f
#define MIC_AGC_SILENCE_RMS       100.0f
#define MIC_AGC_HOLD_BLOCKS       20

static float s_agc_gain = 1.0f;
static int s_agc_hold_blocks = 0;

// weak symbol hooks for streaming
__attribute__((weak)) void asr_ws_send_audio(const void *data, size_t len) { (void)data; (void)len; }
__attribute__((weak)) bool asr_ws_connected(void) { return false; }
__attribute__((weak)) bool asr_ws_streaming(void) { return false; }

static void set_gpio_output(int gpio, int level)
{
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << gpio,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
    gpio_set_level(gpio, level);
}

typedef struct __attribute__((packed)) {
    char riff[4];
    uint32_t size;
    char wave[4];
    char fmt[4];
    uint32_t fmt_size;
    uint16_t audio_format;
    uint16_t num_channels;
    uint32_t sample_rate;
    uint32_t byte_rate;
    uint16_t block_align;
    uint16_t bits_per_sample;
    char data[4];
    uint32_t data_size;
} wav_header_t;

static void write_wav_header(FILE *f, uint32_t sample_rate)
{
    wav_header_t h = {0};
    memcpy(h.riff, "RIFF", 4);
    memcpy(h.wave, "WAVE", 4);
    memcpy(h.fmt,  "fmt ", 4);
    memcpy(h.data, "data", 4);
    h.fmt_size = 16;
    h.audio_format = 1;
    h.num_channels = 1;
    h.sample_rate = sample_rate;
    h.bits_per_sample = 16;
    h.byte_rate = sample_rate * h.num_channels * (h.bits_per_sample/8);
    h.block_align = h.num_channels * (h.bits_per_sample/8);
    h.data_size = 0;
    h.size = 36 + h.data_size;
    fwrite(&h, 1, sizeof(h), f);
}

static void fixup_wav_header(FILE *f)
{
    long end = ftell(f);
    if (end < (long)sizeof(wav_header_t)) return;
    uint32_t data_size = (uint32_t)(end - sizeof(wav_header_t));
    fseek(f, 4, SEEK_SET);
    uint32_t riff_size = 36 + data_size;
    fwrite(&riff_size, 1, 4, f);
    fseek(f, 40, SEEK_SET);
    fwrite(&data_size, 1, 4, f);
    fseek(f, end, SEEK_SET);
}

esp_err_t mic_init(void)
{
    ESP_LOGI(TAG, "Init PDM mic: clk=%d, data=%d, sel=%d, %d Hz", MIC_GPIO_CLK, MIC_GPIO_DATA, MIC_GPIO_SEL, MIC_SAMPLE_RATE);
    set_gpio_output(MIC_GPIO_SEL, 0);

    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, &s_pdm_rx_chan));

    i2s_pdm_rx_config_t pdm_rx_cfg = {
        .clk_cfg = I2S_PDM_RX_CLK_DEFAULT_CONFIG(MIC_SAMPLE_RATE),
        .slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .clk = MIC_GPIO_CLK,
            .din = MIC_GPIO_DATA,
            .invert_flags = { .clk_inv = false }
        }
    };
    ESP_ERROR_CHECK(i2s_channel_init_pdm_rx_mode(s_pdm_rx_chan, &pdm_rx_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(s_pdm_rx_chan));
    return ESP_OK;
}

static char s_current_rec_base[128] = {0};

static void open_new_wav_if_needed(void)
{
    if (s_wav_fp || !s_record_request) return;
    char path[128];
    time_t now = time(NULL);
    struct tm tm;
    localtime_r(&now, &tm);
    strftime(s_current_rec_base, sizeof(s_current_rec_base), "/sdcard/rec-%Y%m%d-%H%M%S", &tm);
    strlcpy(path, s_current_rec_base, sizeof(path));
    strlcat(path, ".wav", sizeof(path));
    s_wav_fp = fopen(path, "wb");
    if (!s_wav_fp) {
        ESP_LOGE(TAG, "Failed to open %s (errno=%d)", path, errno);
        return;
    }
    write_wav_header(s_wav_fp, MIC_SAMPLE_RATE);
    ESP_LOGI(TAG, "Recording to %s", path);
    s_recording_on = true;
}

static void close_wav_if_open(void)
{
    if (!s_wav_fp) return;
    fixup_wav_header(s_wav_fp);
    fclose(s_wav_fp);
    s_wav_fp = NULL;
    s_recording_on = false;
    ESP_LOGI(TAG, "Recording stopped");
}

static inline int16_t saturate_int16(float v)
{
    if (v > 32767.0f) return 32767;
    if (v < -32768.0f) return -32768;
    return (int16_t)lrintf(v);
}

static float compute_rms(const int16_t *samples, size_t count)
{
    if (!samples || count == 0) {
        return 0.0f;
    }
    double acc = 0.0;
    for (size_t i = 0; i < count; ++i) {
        double s = samples[i];
        acc += s * s;
    }
    return (float)sqrt(acc / (double)count);
}

static void agc_process_block(int16_t *samples, size_t count)
{
    if (!samples || count == 0) {
        return;
    }

    float rms = compute_rms(samples, count);
    float desired_gain = s_agc_gain;
    if (rms < MIC_AGC_SILENCE_RMS) {
        if (s_agc_hold_blocks > 0) {
            s_agc_hold_blocks--;
        } else {
            desired_gain = MIC_AGC_MIN_GAIN;
        }
    } else {
        s_agc_hold_blocks = MIC_AGC_HOLD_BLOCKS;
        desired_gain = MIC_AGC_TARGET_RMS / rms;
    }

    desired_gain = fminf(fmaxf(desired_gain, MIC_AGC_MIN_GAIN), MIC_AGC_MAX_GAIN);

    float alpha = (desired_gain > s_agc_gain) ? MIC_AGC_ATTACK_ALPHA : MIC_AGC_RELEASE_ALPHA;
    s_agc_gain = s_agc_gain + alpha * (desired_gain - s_agc_gain);
    s_agc_gain = fminf(fmaxf(s_agc_gain, MIC_AGC_MIN_GAIN), MIC_AGC_MAX_GAIN);

    for (size_t i = 0; i < count; ++i) {
        float amplified = samples[i] * s_agc_gain;
        samples[i] = saturate_int16(amplified);
    }
}

static void mic_capture_task(void *arg)
{
    ESP_LOGI(TAG, "Mic capture task started");
    const size_t buf_bytes = 2048;
    uint8_t *buf = heap_caps_malloc(buf_bytes, MALLOC_CAP_DMA);
    if (!buf) {
        ESP_LOGE(TAG, "No mem for mic buffer");
        vTaskDelete(NULL);
        return;
    }
    while (1) {
        size_t bytes_read = 0;
        esp_err_t ret = i2s_channel_read(s_pdm_rx_chan, buf, buf_bytes, &bytes_read, pdMS_TO_TICKS(1000));

        if (ret == ESP_OK && bytes_read >= sizeof(int16_t)) {
            agc_process_block((int16_t *)buf, bytes_read / sizeof(int16_t));
        }

        if (s_record_request && !s_wav_fp) {
            open_new_wav_if_needed();
        } else if (!s_record_request && s_wav_fp) {
            close_wav_if_open();
        }
        if (ret == ESP_OK && s_wav_fp) {
            fwrite(buf, 1, bytes_read, s_wav_fp);
        }
        if (ret == ESP_OK && asr_ws_connected() && asr_ws_streaming()) {
            asr_ws_send_audio(buf, bytes_read);
        }
    }
}

void mic_start_task(void)
{
    xTaskCreatePinnedToCore(mic_capture_task, "mic_capture", 4096, NULL, 5, NULL, tskNO_AFFINITY);
}

void mic_set_record_request(bool on)
{
    if (!on) {
        // Stop ASR streaming before stopping recording
        asr_ws_request_streaming(false);
        s_record_request = false;
        return;
    }
    // Start streaming if connected when starting to record
    asr_ws_request_streaming(true);
    s_record_request = true;
}

bool mic_is_recording(void)
{
    // Reflect the requested state for UI immediacy
    return s_record_request;
}
