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

#include "cJSON.h"
#include "asr_ws.h"

static const char *TAG = "asr_ws";

typedef enum {
    WS_TASK_IDLE = 0,
    WS_TASK_STARTING,
    WS_TASK_STREAMING,
    WS_TASK_FINISHING,
} ws_task_state_t;

static esp_websocket_client_handle_t s_ws = NULL;
static volatile bool s_ws_connected = false;
static volatile bool s_ws_desired_streaming = false;
static volatile ws_task_state_t s_task_state = WS_TASK_IDLE;
static volatile bool s_pending_finish = false;
static char s_ws_task_id[64] = {0};
static char s_text_buffer[2048];
static size_t s_text_buffer_len = 0;

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

static void ws_reset_task_tracking(void)
{
    s_ws_task_id[0] = '\0';
    s_pending_finish = false;
    s_task_state = WS_TASK_IDLE;
}

static bool ws_task_matches(const char *task_id)
{
    return task_id && s_ws_task_id[0] && strncmp(task_id, s_ws_task_id, sizeof(s_ws_task_id)) == 0;
}

static void ws_maybe_start_task(void);
static void ws_handle_text_message(const char *json_text);

static void ws_send_run_task(void)
{
    if (!s_ws || !s_ws_connected || s_task_state != WS_TASK_IDLE) {
        return;
    }
    memset(s_ws_task_id, 0, sizeof(s_ws_task_id));
    gen_uuid_like(s_ws_task_id, sizeof(s_ws_task_id));
    char json[512];
    int n = snprintf(json, sizeof(json),
        "{\"header\":{\"action\":\"run-task\",\"task_id\":\"%s\",\"streaming\":\"duplex\"},"
        "\"payload\":{\"task_group\":\"audio\",\"task\":\"asr\",\"function\":\"recognition\",\"model\":\"paraformer-realtime-v2\","
        "\"parameters\":{\"format\":\"pcm\",\"sample_rate\":%d,\"inverse_text_normalization_enabled\":true,\"punctuation_prediction_enabled\":true},\"input\":{}}}",
        s_ws_task_id, CONFIG_GEEK_MIC_SAMPLE_RATE);
    if (n > 0 && n < (int)sizeof(json)) {
        if (esp_websocket_client_send_text(s_ws, json, n, portMAX_DELAY) == n) {
            s_task_state = WS_TASK_STARTING;
            s_pending_finish = false;
            ESP_LOGI(TAG, "WS run-task sent (task_id=%s)", s_ws_task_id);
            return;
        }
    }
    ESP_LOGE(TAG, "Failed to send run-task");
    s_ws_task_id[0] = '\0';
}

static void ws_send_finish_task(void)
{
    if (!s_ws || !s_ws_connected || s_task_state != WS_TASK_STREAMING || !s_ws_task_id[0]) {
        return;
    }
    char json[256];
    int n = snprintf(json, sizeof(json),
                     "{\"header\":{\"action\":\"finish-task\",\"task_id\":\"%s\",\"streaming\":\"duplex\"},"
                     "\"payload\":{\"input\":{}}}",
                     s_ws_task_id);
    if (n > 0 && n < (int)sizeof(json)) {
        if (esp_websocket_client_send_text(s_ws, json, n, portMAX_DELAY) == n) {
            s_task_state = WS_TASK_FINISHING;
            s_pending_finish = false;
            ESP_LOGI(TAG, "WS finish-task sent (task_id=%s)", s_ws_task_id);
            return;
        }
    }
    ESP_LOGE(TAG, "Failed to send finish-task");
}

static void ws_maybe_start_task(void)
{
    if (s_ws_desired_streaming && s_ws_connected && s_task_state == WS_TASK_IDLE) {
        ws_send_run_task();
    }
}

static void ws_handle_text_message(const char *json_text)
{
    if (!json_text) {
        return;
    }

    cJSON *root = cJSON_Parse(json_text);
    if (!root) {
        ESP_LOGW(TAG, "WS text parse failed");
        return;
    }

    const cJSON *header = cJSON_GetObjectItemCaseSensitive(root, "header");
    if (!cJSON_IsObject(header)) {
        ESP_LOGD(TAG, "WS message without header");
        cJSON_Delete(root);
        return;
    }

    const cJSON *task_id_item = cJSON_GetObjectItemCaseSensitive(header, "task_id");
    const char *task_id = (cJSON_IsString(task_id_item) && task_id_item->valuestring) ? task_id_item->valuestring : NULL;

    const cJSON *event_item = cJSON_GetObjectItemCaseSensitive(header, "event");
    if (!cJSON_IsString(event_item) || !event_item->valuestring) {
        ESP_LOGD(TAG, "WS message without event");
        cJSON_Delete(root);
        return;
    }

    const char *event_str = event_item->valuestring;
    if (s_ws_task_id[0] != '\0' && !ws_task_matches(task_id)) {
        ESP_LOGW(TAG, "Ignoring event %s for unexpected task_id %s", event_str, task_id ? task_id : "<null>");
        cJSON_Delete(root);
        return;
    }

    if (strcmp(event_str, "task-started") == 0) {
        ESP_LOGI(TAG, "task-started received");
        s_task_state = WS_TASK_STREAMING;
        if (!s_ws_desired_streaming || s_pending_finish) {
            // Request was cancelled while waiting for start.
            ws_send_finish_task();
        }
    } else if (strcmp(event_str, "result-generated") == 0) {
        if (s_task_state == WS_TASK_STARTING) {
            ESP_LOGW(TAG, "Result arrived before task-started; promoting to streaming");
            s_task_state = WS_TASK_STREAMING;
        }

        const cJSON *payload = cJSON_GetObjectItemCaseSensitive(root, "payload");
        if (cJSON_IsObject(payload)) {
            const cJSON *output = cJSON_GetObjectItemCaseSensitive(payload, "output");
            if (cJSON_IsObject(output)) {
                const cJSON *sentence = cJSON_GetObjectItemCaseSensitive(output, "sentence");
                if (cJSON_IsObject(sentence)) {
                    const cJSON *heartbeat = cJSON_GetObjectItemCaseSensitive(sentence, "heartbeat");
                    if (cJSON_IsBool(heartbeat) && cJSON_IsTrue(heartbeat)) {
                        cJSON_Delete(root);
                        return;
                    }

                    const cJSON *sentence_end = cJSON_GetObjectItemCaseSensitive(sentence, "sentence_end");
                    const cJSON *text = cJSON_GetObjectItemCaseSensitive(sentence, "text");
                    if (cJSON_IsString(text) && text->valuestring && cJSON_IsBool(sentence_end) && cJSON_IsTrue(sentence_end)) {
                        ESP_LOGI(TAG, "ASR sentence: %s", text->valuestring);
                    } else if (cJSON_IsString(text) && text->valuestring) {
                        ESP_LOGD(TAG, "ASR partial: %s", text->valuestring);
                    }
                }
            }
        }
    } else if (strcmp(event_str, "task-finished") == 0) {
        ESP_LOGI(TAG, "task-finished received");
        ws_reset_task_tracking();
        ws_maybe_start_task();
    } else if (strcmp(event_str, "task-failed") == 0) {
        const cJSON *code = cJSON_GetObjectItemCaseSensitive(header, "error_code");
        const cJSON *message = cJSON_GetObjectItemCaseSensitive(header, "error_message");
        ESP_LOGE(TAG, "task-failed: code=%s msg=%s", cJSON_IsString(code) && code->valuestring ? code->valuestring : "<none>",
                 cJSON_IsString(message) && message->valuestring ? message->valuestring : "<none>");
        ws_reset_task_tracking();
        ws_maybe_start_task();
    } else {
        ESP_LOGI(TAG, "Unhandled event: %s", event_str);
    }

    cJSON_Delete(root);
}
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
            ws_reset_task_tracking();
            ws_maybe_start_task();
            break;

        case WEBSOCKET_EVENT_DISCONNECTED:
            s_ws_connected = false;
            ws_reset_task_tracking();
            s_text_buffer_len = 0;
            ESP_LOGW(TAG, "WS disconnected");
            if (e) {
                log_ws_error_details(e);
            }
            break;

        case WEBSOCKET_EVENT_DATA: {
            if (!e) break;
            ESP_LOGD(TAG, "WS data: op=0x%02x len=%d total=%d off=%d fin=%d",
                     e->op_code, (int)e->data_len, (int)e->payload_len,
                     (int)e->payload_offset, e->fin);

            if (e->op_code == 0x1) {
                if (e->payload_offset == 0) {
                    s_text_buffer_len = 0;
                }
                if (e->data_len > 0) {
                    if (s_text_buffer_len + e->data_len >= sizeof(s_text_buffer)) {
                        ESP_LOGW(TAG, "WS text frame too large (%zu + %d)", s_text_buffer_len, (int)e->data_len);
                        s_text_buffer_len = 0;
                    } else {
                        memcpy(s_text_buffer + s_text_buffer_len, e->data_ptr, e->data_len);
                        s_text_buffer_len += e->data_len;
                        if (e->fin && (size_t)(e->payload_offset + e->data_len) == e->payload_len) {
                            s_text_buffer[s_text_buffer_len] = '\0';
                            int show = s_text_buffer_len > 160 ? 160 : (int)s_text_buffer_len;
                            ESP_LOGD(TAG, "WS text: %.*s%s", show, s_text_buffer,
                                     (s_text_buffer_len > (size_t)show ? "..." : ""));
                            ws_handle_text_message(s_text_buffer);
                            s_text_buffer_len = 0;
                        }
                    }
                }
            } else if (e->op_code == 0x2) {
                ESP_LOGD(TAG, "WS bin frame %d bytes", (int)e->data_len);
            } else if (e->op_code == 0x8) {
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
            ws_reset_task_tracking();
            s_text_buffer_len = 0;
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
bool asr_ws_streaming(void) { return s_task_state == WS_TASK_STREAMING; }

void asr_ws_send_audio(const void *data, size_t len)
{
    if (s_ws && s_ws_connected && s_task_state == WS_TASK_STREAMING && data && len) {
        esp_websocket_client_send_bin(s_ws, (const char *)data, len, portMAX_DELAY);
    }
}

void asr_ws_request_streaming(bool on)
{
    s_ws_desired_streaming = on;
    if (!s_ws_connected) return; // will take effect upon next connect
    if (on) {
        ws_maybe_start_task();
    } else {
        if (s_task_state == WS_TASK_STREAMING) {
            ws_send_finish_task();
        } else if (s_task_state == WS_TASK_STARTING) {
            s_pending_finish = true;
        } else if (s_task_state == WS_TASK_IDLE) {
            s_pending_finish = false;
        }
    }
}
