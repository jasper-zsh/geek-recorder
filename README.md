# Geek Recorder (ESP32-S3)

Minimal firmware scaffold for a real-time recorder with:
- PDM silicon mic (I2S PDM RX) on GPIO SEL=6, CLK=14, DATA=13
- 1.14" 135x240 SPI LCD (ST7789) on DC=8, CS=10, CLK=12, MOSI=11, RST=9, BL=7
- TF card via SDMMC (4-bit) on CLK=36, CMD=35, D0=37, D1=33, D2=38, D3=34
- Wi-Fi AP provisioning + HTTP file manager

This scaffold brings up the peripherals, starts a simple HTTP server, records mic audio to WAV on the TF card, connects to Aliyun Paraformer realtime ASR via WebSocket, and streams PCM for transcription. The screen currently shows a basic banner and last line placeholder; richer text rendering will be added next.

## Build

- Install ESP-IDF v5.x and set up the environment.
- Set target once: `idf.py set-target esp32s3`
- Build and flash: `idf.py -p <PORT> flash monitor`

## Configure pins or options

Use `idf.py menuconfig` → Geek Recorder Configuration to adjust pins and settings. Defaults match the design doc.

## Provisioning and file management

- Device starts an open AP named `GeekRecorder-XXYY`.
- Connect and open `http://192.168.4.1/` to submit Wi-Fi SSID/password and API key (stored in NVS).
- A simple file browser lists `/sdcard` with download and delete.

## Realtime ASR (Aliyun Paraformer)

- Put your DashScope API Key via the web page on `/` (stored in NVS).
- Device opens a WebSocket to `wss://dashscope.aliyuncs.com/api-ws/v1/inference` with `Authorization: bearer <API Key>`.
- It sends `run-task` for `paraformer-realtime-v2` at 16 kHz PCM, then streams live mic frames. Results are logged and appended to a `rec-*.txt` file beside the WAV whenever a sentence ends.
- Note: TLS verification relies on ESP-IDF CA store; ensure time is reasonable for TLS if using SNTP.

## Chinese Font on SD Card

- Generate a LVGL binary font with LVGL Font Converter (`--format lvgl --bpp 4 --size 16 --range 0x20-0x7F,common-CJK`).
- Save as `/sdcard/fonts/chs_16.bin`. On boot, the firmware registers an LVGL FS driver `S:` mapped to `/sdcard` and tries to load `S:/fonts/chs_16.bin` via `lv_font_load`.
- If found, the UI label switches to this font and can render Chinese; otherwise it falls back to the built‑in Latin font or ASCII console.

## Notes

- UI now supports LVGL: color UI with wrapping and scrolling text label; Chinese requires a CJK font (see `main/fonts/README.md`). Fallback ASCII console remains available if LVGL is disabled.
- PDM RX uses the modern `i2s_pdm_rx` path (`i2s_channel_init_pdm_rx_mode`). Audio is written to rotating WAV files `/sdcard/rec-YYYYMMDD-HHMMSS.wav`.
- TF card is mounted as `/sdcard` using SDMMC 4-bit with custom pins.

## Next steps (per design)

- WebSocket client: stream PCM frames to Aliyun Paraformer realtime ASR and parse responses.
- Improve LCD rendering: add Chinese LVGL font via converter and style the transcript view; optionally add u8g2 monochrome fallback for ultra‑low RAM builds.
- Captive portal for provisioning (optional refinement).
- Persist and rotate transcripts alongside audio in `/sdcard`.

---
This project follows the hardware definition and uses ESP-IDF components: `esp_lcd`, `i2s_pdm`, `sdmmc`, `esp_http_server`, `esp_websocket_client`.
