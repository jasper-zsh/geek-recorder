#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_event.h"
#include "esp_websocket_client.h"
#include "esp_crt_bundle.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "asr_ws.h"

static const char *TAG = "asr_ws";

static esp_websocket_client_handle_t s_ws = NULL;
static volatile bool s_ws_connected = false;
static volatile bool s_ws_task_started = false;
static char s_ws_task_id[64] = {0};

static uint32_t urand32(void) { return esp_random(); }
static void gen_uuid_like(char *out, size_t out_sz)
{
    const char *hex = "0123456789abcdef";
    size_t j=0;
    for (int i=0;i<36 && j+1<out_sz;i++){
        bool dash = (i==8||i==13||i==18||i==23);
        if (dash){ out[j++]='-'; continue; }
        uint32_t r = urand32();
        out[j++] = hex[(r>>((i&3)*4))&0xF];
    }
    out[j]=0;
}

static void ws_send_run_task(void)
{
    if (!s_ws || !s_ws_connected) return;
    memset(s_ws_task_id, 0, sizeof(s_ws_task_id));
    gen_uuid_like(s_ws_task_id, sizeof(s_ws_task_id));
    char json[512];
    int n = snprintf(json, sizeof(json),
        "{\"header\":{\"action\":\"run-task\",\"task_id\":\"%s\",\"streaming\":\"duplex\"},"
        "\"payload\":{\"task_group\":\"audio\",\"task\":\"asr\",\"function\":\"recognition\",\"model\":\"paraformer-realtime-v2\","
        "\"parameters\":{\"format\":\"pcm\",\"sample_rate\":%d,\"inverse_text_normalization_enabled\":true,\"punctuation_prediction_enabled\":true},\"input\":{}}}",
        s_ws_task_id, CONFIG_GEEK_MIC_SAMPLE_RATE);
    if (n > 0 && n < (int)sizeof(json)) {
        esp_websocket_client_send_text(s_ws, json, n, portMAX_DELAY);
        ESP_LOGI(TAG, "WS run-task sent");
    }
}

// finish-task API 暂不需要；如需在停止流时显式结束可再加入

static const char *ws_event_name(int32_t event_id)
{
    switch (event_id) {
    case WEBSOCKET_EVENT_BEGIN:          return "BEGIN";
    case WEBSOCKET_EVENT_BEFORE_CONNECT: return "BEFORE_CONNECT";
    case WEBSOCKET_EVENT_CONNECTED:      return "CONNECTED";
    case WEBSOCKET_EVENT_DISCONNECTED:   return "DISCONNECTED";
    case WEBSOCKET_EVENT_DATA:           return "DATA";
    case WEBSOCKET_EVENT_CLOSED:         return "CLOSED";
    case WEBSOCKET_EVENT_ERROR:          return "ERROR";
    case WEBSOCKET_EVENT_FINISH:         return "FINISH";
    default:                             return "UNKNOWN";
    }
}

static void log_ws_error_details(const esp_websocket_event_data_t *e)
{
    if (!e) return;
    ESP_LOGE(TAG, "WS error: http_status=%d type=%d",
             e->error_handle.esp_ws_handshake_status_code,
             e->error_handle.error_type);
    if (e->error_handle.error_type == WEBSOCKET_ERROR_TYPE_TCP_TRANSPORT) {
        ESP_LOGE(TAG, "  esp_tls_last_esp_err=0x%x tls_stack_err=0x%x sock_errno=%d",
                 e->error_handle.esp_tls_last_esp_err,
                 e->error_handle.esp_tls_stack_err,
                 e->error_handle.esp_transport_sock_errno);
    }
}

static void ws_event_handler(void *arg, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_websocket_event_data_t *e = (esp_websocket_event_data_t *)event_data;
    ESP_LOGI(TAG, "WS event: base=%s id=%d (%s)", base, (int)event_id, ws_event_name(event_id));

    switch (event_id) {
        case WEBSOCKET_EVENT_BEGIN:
            // client thread started
            break;

        case WEBSOCKET_EVENT_BEFORE_CONNECT:
            // about to connect; useful when diagnosing handshake issues
            break;

        case WEBSOCKET_EVENT_CONNECTED:
            s_ws_connected = true;
            ESP_LOGI(TAG, "WS connected");
            ws_send_run_task();
            break;

        case WEBSOCKET_EVENT_DISCONNECTED:
            s_ws_connected = false;
            s_ws_task_started = false;
            ESP_LOGW(TAG, "WS disconnected");
            if (e) {
                log_ws_error_details(e);
            }
            break;

        case WEBSOCKET_EVENT_DATA: {
            if (!e) break;
            ESP_LOGI(TAG, "WS data: op=0x%02x len=%d total=%d off=%d fin=%d",
                     e->op_code, (int)e->data_len, (int)e->payload_len,
                     (int)e->payload_offset, e->fin);

            // If text frame, try to detect task-start quickly (keep log concise)
            if (e->op_code == 0x1 && e->data_len > 0) {
                const char *txt = (const char *)e->data_ptr;
                if (txt && strstr(txt, "\"started\":true")) {
                    s_ws_task_started = true;
                    ESP_LOGI(TAG, "WS task-started acknowledged");
                }
                // Show preview of text frame (truncated)
                int show = e->data_len > 160 ? 160 : e->data_len;
                ESP_LOGD(TAG, "WS text: %.*s%s", show, txt, (e->data_len > show ? "..." : ""));
            } else if (e->op_code == 0x2) {
                // binary
                ESP_LOGD(TAG, "WS bin frame %d bytes", (int)e->data_len);
            } else if (e->op_code == 0x8) {
                // close frame
                if (e->data_len >= 2) {
                    int code = 256 * ((const uint8_t*)e->data_ptr)[0] + ((const uint8_t*)e->data_ptr)[1];
                    ESP_LOGW(TAG, "WS close frame: code=%d", code);
                } else {
                    ESP_LOGW(TAG, "WS close frame");
                }
            } else if (e->op_code == 0xA) {
                ESP_LOGD(TAG, "WS PONG");
            }
            break;
        }

        case WEBSOCKET_EVENT_ERROR:
            log_ws_error_details(e);
            break;

        case WEBSOCKET_EVENT_CLOSED:
            ESP_LOGW(TAG, "WS closed cleanly");
            break;

        case WEBSOCKET_EVENT_FINISH:
            ESP_LOGI(TAG, "WS finished");
            break;

        default:
            ESP_LOGW(TAG, "WS unknown event: %d", (int)event_id);
            break;
    }
}

static void ws_task(void *arg)
{
    // Read API key from NVS
    nvs_handle_t n;
    char api_key[129] = {0};
    size_t len = sizeof(api_key);
    if (nvs_open("app", NVS_READONLY, &n) == ESP_OK) {
        nvs_get_str(n, "api_key", api_key, &len);
        nvs_close(n);
    }
    if (strlen(api_key) == 0) {
        ESP_LOGW(TAG, "No API key; WS not started");
        vTaskDelete(NULL);
        return;
    }

    // Build additional handshake headers.
    // DashScope 文档示例使用小写 bearer，这里与其保持一致；并确保每行以 CRLF 结尾。
    char auth_hdr[320];
    if (strlen(CONFIG_GEEK_WS_ORIGIN)) {
        snprintf(auth_hdr, sizeof(auth_hdr),
                 "Authorization: bearer %s\r\nOrigin: %s\r\n",
                 api_key, CONFIG_GEEK_WS_ORIGIN);
    } else {
        snprintf(auth_hdr, sizeof(auth_hdr),
                 "Authorization: bearer %s\r\n",
                 api_key);
    }

    esp_websocket_client_config_t cfg = {
        .uri = CONFIG_GEEK_WS_URI,
        .headers = auth_hdr,
        .subprotocol = (strlen(CONFIG_GEEK_WS_SUBPROTOCOL) ? CONFIG_GEEK_WS_SUBPROTOCOL : NULL),
        .network_timeout_ms = 10000,
        .reconnect_timeout_ms = 5000,
        .crt_bundle_attach = esp_crt_bundle_attach,
    };
    // Log sanitized handshake info for debugging
    size_t klen = strlen(api_key);
    char tail[9] = {0};
    if (klen >= 4) strncpy(tail, api_key + klen - 4, 4);
    ESP_LOGI(TAG, "WS init start");
    s_ws = esp_websocket_client_init(&cfg);
    ESP_LOGI(TAG, "WS init: uri=%s, origin=%s, subprotocol=%s, auth=bearer ***%s",
             CONFIG_GEEK_WS_URI,
             strlen(CONFIG_GEEK_WS_ORIGIN) ? CONFIG_GEEK_WS_ORIGIN : "<none>",
             strlen(CONFIG_GEEK_WS_SUBPROTOCOL) ? CONFIG_GEEK_WS_SUBPROTOCOL : "<none>",
             tail);
    esp_websocket_register_events(s_ws, WEBSOCKET_EVENT_ANY, ws_event_handler, NULL);
    esp_websocket_client_start(s_ws);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void asr_ws_start(void)
{
    static bool started = false;
    if (!started) {
        started = true;
        xTaskCreatePinnedToCore(ws_task, "ws_asr", 6144, NULL, 5, NULL, tskNO_AFFINITY);
    }
}

bool asr_ws_connected(void) { return s_ws_connected; }
bool asr_ws_streaming(void) { return s_ws_task_started; }

void asr_ws_send_audio(const void *data, size_t len)
{
    if (s_ws && s_ws_connected && s_ws_task_started && data && len) {
        esp_websocket_client_send_bin(s_ws, (const char *)data, len, portMAX_DELAY);
    }
}
