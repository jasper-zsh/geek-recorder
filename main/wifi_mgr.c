#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "wifi_mgr.h"
#include "esp_mac.h"

static const char *TAG = "wifi_mgr";
static char s_target_ssid[33] = {0};
static bool s_logged_creds_once = false;

#define NVS_NS_WIFI   "wifi"
#define NVS_KEY_SSID  "ssid"
#define NVS_KEY_PASS  "pass"

#ifndef CONFIG_GEEK_AP_PASSWORD
#define CONFIG_GEEK_AP_PASSWORD ""
#endif
#define AP_PASSWORD       CONFIG_GEEK_AP_PASSWORD

static bool s_provisioning = false;
static SemaphoreHandle_t s_scan_mutex = NULL;
static volatile bool s_scan_busy = false;
static wifi_ap_record_t *s_scan_results = NULL;
static uint16_t s_scan_count = 0;
static esp_timer_handle_t s_reconnect_timer = NULL;

static void reconnect_timer_cb(void *arg)
{
    wifi_ap_record_t ap;
    if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
        // Already connected; no action
        return;
    }
    ESP_LOGI(TAG, "Reconnecting to '%s'...", s_target_ssid);
    esp_wifi_connect();
}

static const char *wifi_reason_to_str(uint8_t r)
{
    switch (r) {
        case 1:   return "UNSPECIFIED";
        case 2:   return "AUTH_EXPIRE";
        case 3:   return "AUTH_LEAVE";
        case 4:   return "ASSOC_EXPIRE";
        case 5:   return "ASSOC_TOOMANY";
        case 6:   return "CLASS2_FROM_NONAUTH";
        case 7:   return "CLASS3_FROM_NONASSOC";
        case 8:   return "DISASSOC_LEAVE";
        case 10:  return "DISASSOC_PWRCAP_BAD";
        case 11:  return "DISASSOC_SUPCHAN_BAD";
        case 13:  return "IE_INVALID";
        case 14:  return "MIC_FAILURE/4WAY_TIMEOUT";
        case 15:  return "4WAY_HANDSHAKE_TIMEOUT";
        case 18:  return "GROUP_CIPHER_INVALID";
        case 19:  return "PAIRWISE_CIPHER_INVALID";
        case 23:  return "802_1X_AUTH_FAILED";
        case 29:  return "BAD_CIPHER_OR_AKM";
        case 32:  return "UNSPECIFIED_QOS";
        case 33:  return "NOT_ENOUGH_BANDWIDTH";
        case 67:  return "TX_LINK_ESTABLISHMENT_FAILED";
        case 68:  return "ALTERNATIVE_CHANNEL_OCCUPIED";
        case 200: return "BEACON_TIMEOUT";
        case 201: return "NO_AP_FOUND";
        case 202: return "AUTH_FAIL";
        case 203: return "ASSOC_FAIL";
        case 204: return "HANDSHAKE_TIMEOUT";
        case 205: return "CONNECTION_FAIL";
        case 210: return "SECURITY_MISMATCH/ESP_SPECIFIC";
        case 212: return "NO_AP_FOUND_RSSI";
        default:  return "UNKNOWN";
    }
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        if (!s_provisioning) {
            ESP_LOGI(TAG, "STA start: preparing to connect SSID='%s'", s_target_ssid);
            // One-time pre-scan for target SSID to learn auth/channel and pick best BSSID
            if (s_target_ssid[0]) {
                wifi_scan_config_t sc = {
                    .ssid = (const uint8_t *)s_target_ssid,
                    .bssid = 0,
                    .channel = 0,
                    .show_hidden = false,
                    .scan_type = WIFI_SCAN_TYPE_ACTIVE,
                    .scan_time = { .active = { .min = 60, .max = 150 } },
                };
                if (esp_wifi_scan_start(&sc, true) == ESP_OK) {
                    uint16_t n = 0; esp_wifi_scan_get_ap_num(&n);
                    if (n) {
                        wifi_ap_record_t *recs = malloc(sizeof(wifi_ap_record_t) * n);
                        if (recs && esp_wifi_scan_get_ap_records(&n, recs) == ESP_OK) {
                            wifi_ap_record_t best = {0}; bool found=false;
                            for (uint16_t i=0;i<n;i++) {
                                if (strncmp((const char*)recs[i].ssid, s_target_ssid, sizeof(recs[i].ssid)) != 0) continue;
                                if (!found || recs[i].rssi > best.rssi) { best = recs[i]; found = true; }
                            }
                            if (found) {
                                ESP_LOGI(TAG,
                                    "Found '%s': ch=%u rssi=%d auth=%d pairwise=%d group=%d",
                                    best.ssid, best.primary, best.rssi,
                                    best.authmode, best.pairwise_cipher, best.group_cipher);
                                wifi_config_t sta = {0};
                                esp_wifi_get_config(WIFI_IF_STA, &sta);
                                // Do not hard-lock BSSID/channel to avoid 'no suitable AP' and AUTH_LEAVE
                                sta.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
                                sta.sta.threshold.authmode = WIFI_AUTH_OPEN;
                                sta.sta.pmf_cfg.capable = true;
                                sta.sta.pmf_cfg.required = false;
#ifdef WIFI_WPA3_SAE_PWE_BOTH
                                sta.sta.sae_pwe_h2e = WIFI_WPA3_SAE_PWE_BOTH;
#endif
                                esp_wifi_set_config(WIFI_IF_STA, &sta);
                            }
                        }
                        if (recs) free(recs);
                    } else {
                        ESP_LOGW(TAG, "Pre-scan: target SSID not found");
                    }
                }
            }
            ESP_LOGI(TAG, "Connecting to SSID='%s'...", s_target_ssid);
            esp_wifi_connect();
        } else {
            ESP_LOGI(TAG, "Provisioning: skip STA connect");
        }
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *e = (wifi_event_sta_disconnected_t *)event_data;
        if (e) {
            ESP_LOGW(TAG,
                     "STA disconnected, reason=%u (%s), ssid=%.*s, bssid=%02x:%02x:%02x:%02x:%02x:%02x",
                     e->reason, wifi_reason_to_str(e->reason),
                     e->ssid_len, e->ssid,
                     e->bssid[0], e->bssid[1], e->bssid[2], e->bssid[3], e->bssid[4], e->bssid[5]);
        } else {
            ESP_LOGW(TAG, "STA disconnected, reason=<null>");
        }
        if (!s_provisioning) {
            // If security mismatch (e.g., 210), do a focused scan to learn AP's auth and reconfigure accordingly
            if (e && e->reason == 210 && s_target_ssid[0]) {
                wifi_scan_config_t sc = {
                    .ssid = (const uint8_t *)s_target_ssid,
                    .bssid = 0,
                    .channel = 0,
                    .show_hidden = false,
                    .scan_type = WIFI_SCAN_TYPE_ACTIVE,
                    .scan_time = { .active = { .min = 60, .max = 150 } },
                };
                if (esp_wifi_scan_start(&sc, true) == ESP_OK) {
                    uint16_t n = 0; esp_wifi_scan_get_ap_num(&n);
                    if (n) {
                        wifi_ap_record_t *recs = malloc(sizeof(wifi_ap_record_t) * n);
                        if (recs && esp_wifi_scan_get_ap_records(&n, recs) == ESP_OK) {
                            wifi_ap_record_t best = {0}; bool found=false;
                            for (uint16_t i=0;i<n;i++) {
                                if (strncmp((const char*)recs[i].ssid, s_target_ssid, sizeof(recs[i].ssid)) != 0) continue;
                                if (!found || recs[i].rssi > best.rssi) { best = recs[i]; found = true; }
                            }
                            if (found) {
                                ESP_LOGW(TAG, "Retry config for '%s': auth=%d pairwise=%d group=%d ch=%u",
                                         best.ssid, best.authmode, best.pairwise_cipher, best.group_cipher, best.primary);
                                wifi_config_t sta = {0};
                                esp_wifi_get_config(WIFI_IF_STA, &sta);
                                // Do not lock BSSID/channel; just allow all-channel scan
                                sta.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
                                // Default: do not filter by auth mode to avoid filtering out transition modes
                                sta.sta.threshold.authmode = WIFI_AUTH_OPEN;
                                // PMF capability is necessary for WPA3; require PMF only if AP advertises WPA3
#ifdef WIFI_AUTH_WPA3_PSK
                                if (best.authmode == WIFI_AUTH_WPA3_PSK
    #ifdef WIFI_AUTH_WPA2_WPA3_PSK
                                    || best.authmode == WIFI_AUTH_WPA2_WPA3_PSK
    #endif
                                ) {
                                    sta.sta.pmf_cfg.capable = true;
                                    sta.sta.pmf_cfg.required = true; // WPA3 mandates PMF
    #if defined(WIFI_SAE_PWE_BOTH)
                                    sta.sta.sae_pwe_h2e = WIFI_SAE_PWE_BOTH;
    #elif defined(WPA3_SAE_PWE_BOTH)
                                    sta.sta.sae_pwe_h2e = WPA3_SAE_PWE_BOTH;
    #endif
                                } else
#endif
                                {
                                    sta.sta.pmf_cfg.capable = true;
                                    sta.sta.pmf_cfg.required = false;
                                }
                                esp_wifi_set_config(WIFI_IF_STA, &sta);
                            }
                        }
                        if (recs) free(recs);
                    }
                }
            }
            // Schedule a deferred reconnect that cancels itself if we are already connected
            if (!s_reconnect_timer) {
                const esp_timer_create_args_t targs = {
                    .callback = &reconnect_timer_cb,
                    .arg = NULL,
                    .dispatch_method = ESP_TIMER_TASK,
                    .name = "wifi_reconnect",
                };
                esp_timer_create(&targs, &s_reconnect_timer);
            }
            if (s_reconnect_timer) {
                esp_timer_stop(s_reconnect_timer);
                esp_timer_start_once(s_reconnect_timer, 1500ULL * 1000ULL);
            }
        } else {
            ESP_LOGI(TAG, "Provisioning: skip STA reconnect");
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        // Cancel pending reconnect timer (if any)
        if (s_reconnect_timer) esp_timer_stop(s_reconnect_timer);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_LOST_IP) {
        ESP_LOGW(TAG, "Lost IP");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_SCAN_DONE) {
        uint16_t ap_num = 0;
        esp_wifi_scan_get_ap_num(&ap_num);
        wifi_ap_record_t *recs = NULL;
        if (ap_num) {
            recs = malloc(sizeof(wifi_ap_record_t) * ap_num);
            if (recs) {
                esp_wifi_scan_get_ap_records(&ap_num, recs);
            } else {
                ap_num = 0;
            }
        }
        if (!s_scan_mutex) s_scan_mutex = xSemaphoreCreateMutex();
        if (s_scan_mutex) xSemaphoreTake(s_scan_mutex, portMAX_DELAY);
        if (s_scan_results) free(s_scan_results);
        s_scan_results = recs;
        s_scan_count = ap_num;
        s_scan_busy = false;
        if (s_scan_mutex) xSemaphoreGive(s_scan_mutex);
        ESP_LOGI(TAG, "Scan done: %u AP(s)", (unsigned)ap_num);
    }
}

esp_err_t wifi_mgr_init(void)
{
    bool have_creds = false;
    char ssid[33] = {0};
    char pass[65] = {0};
    size_t len;

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_LOST_IP, &wifi_event_handler, NULL));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    // Do NOT force a fixed country. Use AUTO so the STA follows AP's country
    // information to avoid regulatory mismatches that may cause disconnect loops.
    wifi_country_t c = { .cc = "US", .schan = 1, .nchan = 11, .policy = WIFI_COUNTRY_POLICY_AUTO };
    esp_wifi_set_country(&c);
    ESP_LOGI(TAG, "Wi-Fi country policy AUTO (cc=%s ch=%d-%d)", c.cc, c.schan, c.schan + c.nchan - 1);

    nvs_handle_t n;
    if (nvs_open(NVS_NS_WIFI, NVS_READONLY, &n) == ESP_OK) {
        len = sizeof(ssid);
        if (nvs_get_str(n, NVS_KEY_SSID, ssid, &len) == ESP_OK && strlen(ssid) > 0) {
            len = sizeof(pass);
            if (nvs_get_str(n, NVS_KEY_PASS, pass, &len) == ESP_OK) {
                if (strlen(pass) >= 8 || strlen(pass) == 0) {
                    have_creds = true;
                    strlcpy(s_target_ssid, ssid, sizeof s_target_ssid);
#if CONFIG_GEEK_DEBUG_LOG_WIFI_CREDENTIALS
                    if (!s_logged_creds_once) {
                        ESP_LOGW(TAG, "DEBUG Wi-Fi creds from NVS: SSID='%s' PASS='%s' (len=%u)",
                                 ssid, pass, (unsigned)strlen(pass));
                        s_logged_creds_once = true;
                    }
#endif
                }
            }
        }
        nvs_close(n);
    }

    s_provisioning = !have_creds;
    if (s_provisioning) {
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
        ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
        // AP config
        esp_netif_create_default_wifi_ap();
        wifi_config_t ap = {0};
        uint8_t mac[6];
        esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
        snprintf((char*)ap.ap.ssid, sizeof(ap.ap.ssid), "GeekRecorder-%02X%02X", mac[4], mac[5]);
        ap.ap.ssid_len = strlen((char*)ap.ap.ssid);
        ap.ap.channel = 1;
        ap.ap.max_connection = 4;
        if (strlen(AP_PASSWORD) >= 8) {
            ap.ap.authmode = WIFI_AUTH_WPA2_PSK;
            strncpy((char*)ap.ap.password, AP_PASSWORD, sizeof(ap.ap.password));
        } else {
            ap.ap.authmode = WIFI_AUTH_OPEN;
        }
        ap.ap.ssid_hidden = false;
        ap.ap.beacon_interval = 100;
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap));
        ESP_LOGI(TAG, "Provisioning AP SSID=%s", ap.ap.ssid);
        // STA interface for scanning
        esp_netif_create_default_wifi_sta();
    } else {
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        esp_netif_create_default_wifi_sta();
        wifi_config_t sta = {0};
        strncpy((char*)sta.sta.ssid, ssid, sizeof(sta.sta.ssid));
        strncpy((char*)sta.sta.password, pass, sizeof(sta.sta.password));
        // 不按加密方式做最小阈值过滤，避免遇到厂商扩展的“安全策略不匹配(210)”
        sta.sta.threshold.authmode = WIFI_AUTH_OPEN;
        // PMF能力开启但不强制，兼容更多AP
        sta.sta.pmf_cfg.capable = true;
        sta.sta.pmf_cfg.required = false;
        sta.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
        sta.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
        sta.sta.listen_interval = 3;
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta));
        // 关闭省电以避免部分AP上出现Beacon丢失导致的反复掉线
        esp_wifi_set_ps(WIFI_PS_NONE);
        // 推荐的 11b/g/n + HT20 设置，提升兼容性
        esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
        esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT20);
    }
    ESP_ERROR_CHECK(esp_wifi_start());
    // In STA mode, rely on WIFI_EVENT_STA_START to trigger connect
    return ESP_OK;
}

bool wifi_mgr_in_provisioning(void)
{
    return s_provisioning;
}

esp_err_t wifi_mgr_enter_provisioning(void)
{
    // Switch to AP+STA without erasing NVS; reuse same SoftAP naming logic
    s_provisioning = true;
    ESP_ERROR_CHECK(esp_wifi_stop());
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    esp_netif_create_default_wifi_ap();
    // Configure AP
    wifi_config_t ap = {0};
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
    snprintf((char*)ap.ap.ssid, sizeof(ap.ap.ssid), "GeekRecorder-%02X%02X", mac[4], mac[5]);
    ap.ap.ssid_len = strlen((char*)ap.ap.ssid);
    ap.ap.channel = 1;
    ap.ap.max_connection = 4;
    if (strlen(AP_PASSWORD) >= 8) {
        ap.ap.authmode = WIFI_AUTH_WPA2_PSK;
        strncpy((char*)ap.ap.password, AP_PASSWORD, sizeof(ap.ap.password));
    } else {
        ap.ap.authmode = WIFI_AUTH_OPEN;
    }
    ap.ap.ssid_hidden = false;
    ap.ap.beacon_interval = 100;
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap));
    ESP_LOGI(TAG, "Enter provisioning AP SSID=%s", ap.ap.ssid);
    // Ensure STA netif is present for scans and future connects
    esp_netif_create_default_wifi_sta();
    ESP_ERROR_CHECK(esp_wifi_start());
    return ESP_OK;
}

esp_err_t wifi_mgr_scan_start(void)
{
    if (!s_scan_mutex) s_scan_mutex = xSemaphoreCreateMutex();
    if (s_scan_mutex) xSemaphoreTake(s_scan_mutex, portMAX_DELAY);
    esp_wifi_scan_stop();
    if (s_scan_results) { free(s_scan_results); s_scan_results = NULL; s_scan_count = 0; }
    wifi_scan_config_t scan = {
        .ssid = 0,
        .bssid = 0,
        .channel = 0,
        .show_hidden = false,
        .scan_type = WIFI_SCAN_TYPE_ACTIVE,
        .scan_time = { .active = { .min = 30, .max = 60 } },
    };
    esp_err_t err = esp_wifi_scan_start(&scan, false);
    if (err == ESP_OK) s_scan_busy = true;
    if (s_scan_mutex) xSemaphoreGive(s_scan_mutex);
    return err;
}

void wifi_mgr_scan_status(bool *pending, const wifi_ap_record_t **recs, uint16_t *count)
{
    if (!s_scan_mutex) s_scan_mutex = xSemaphoreCreateMutex();
    if (s_scan_mutex) xSemaphoreTake(s_scan_mutex, portMAX_DELAY);
    if (pending) *pending = s_scan_busy;
    if (recs) *recs = s_scan_results;
    if (count) *count = s_scan_count;
    if (s_scan_mutex) xSemaphoreGive(s_scan_mutex);
}

esp_err_t wifi_mgr_apply_provision(const char *ssid, const char *pass, bool *switched_to_sta)
{
    if (switched_to_sta) *switched_to_sta = false;
    if (!ssid || strlen(ssid) == 0) return ESP_ERR_INVALID_ARG;
    if (strlen(pass) > 0 && strlen(pass) < 8) return ESP_ERR_INVALID_SIZE;

    nvs_handle_t n;
    if (nvs_open(NVS_NS_WIFI, NVS_READWRITE, &n) == ESP_OK) {
        nvs_set_str(n, NVS_KEY_SSID, ssid);
        nvs_set_str(n, NVS_KEY_PASS, pass ? pass : "");
        nvs_commit(n);
        nvs_close(n);
    }
#if CONFIG_GEEK_DEBUG_LOG_WIFI_CREDENTIALS
    if (!s_logged_creds_once) {
        ESP_LOGW(TAG, "DEBUG Applying Wi-Fi creds: SSID='%s' PASS='%s' (len=%u)",
                 ssid, pass ? pass : "", (unsigned)(pass ? strlen(pass) : 0));
        s_logged_creds_once = true;
    }
#endif
    // Pre-scan target SSID to improve reliability and 2.4G validation
    uint16_t ap_num = 0;
    wifi_ap_record_t *recs = NULL;
    wifi_scan_config_t sc = {
        .ssid = (const uint8_t *)ssid,
        .bssid = 0,
        .channel = 0,
        .show_hidden = false,
        .scan_type = WIFI_SCAN_TYPE_ACTIVE,
        .scan_time = { .active = { .min = 80, .max = 180 } },
    };
    esp_wifi_scan_stop();
    esp_err_t se = esp_wifi_scan_start(&sc, true);
    if (se == ESP_OK) {
        esp_wifi_scan_get_ap_num(&ap_num);
        if (ap_num) {
            recs = malloc(sizeof(wifi_ap_record_t) * ap_num);
            if (recs) esp_wifi_scan_get_ap_records(&ap_num, recs);
        }
    } else {
        ESP_LOGW(TAG, "Pre-scan failed: %s", esp_err_to_name(se));
    }
    // If not found, try scanning hidden SSIDs once
    if (ap_num == 0) {
        sc.show_hidden = true;
        se = esp_wifi_scan_start(&sc, true);
        if (se == ESP_OK) {
            esp_wifi_scan_get_ap_num(&ap_num);
            if (ap_num) {
                recs = malloc(sizeof(wifi_ap_record_t) * ap_num);
                if (recs) esp_wifi_scan_get_ap_records(&ap_num, recs);
            }
        }
    }

    // Choose strongest BSSID of the target SSID if found
    wifi_ap_record_t best = {0};
    bool found = false;
    for (uint16_t i = 0; i < ap_num; ++i) {
        if (strncmp((const char*)recs[i].ssid, ssid, sizeof(best.ssid)) == 0) {
            if (!found || recs[i].rssi > best.rssi) { best = recs[i]; found = true; }
        }
    }
    if (recs) free(recs);
    if (found) {
        ESP_LOGI(TAG, "Found SSID '%s': ch=%u rssi=%d auth=%d", ssid, best.primary, best.rssi, best.authmode);
    } else {
        ESP_LOGW(TAG, "SSID '%s' not found in 2.4G scan. It may be 5GHz/hidden/or ch12/13.", ssid);
    }

    // Switch to STA only
    wifi_config_t sta = {0};
    strncpy((char*)sta.sta.ssid, ssid, sizeof(sta.sta.ssid));
    strncpy((char*)sta.sta.password, pass ? pass : "", sizeof(sta.sta.password));
    // Always allow any auth (avoid filtering by auth mode). Do not lock BSSID/channel.
    sta.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
    sta.sta.threshold.authmode = WIFI_AUTH_OPEN; // do not filter by auth mode
    // PMF/SAE: enable capability, but not required, and support both SAE PWE methods if available
    sta.sta.pmf_cfg.capable = true;
    sta.sta.pmf_cfg.required = false;
#ifdef WIFI_WPA3_SAE_PWE_BOTH
    sta.sta.sae_pwe_h2e = WIFI_WPA3_SAE_PWE_BOTH;
#endif
    sta.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
    sta.sta.listen_interval = 3;

    s_provisioning = false;
    strlcpy(s_target_ssid, ssid, sizeof s_target_ssid);
    ESP_LOGI(TAG, "Applying credentials for '%s', restarting STA...", s_target_ssid);
    // Cleanly restart Wi-Fi STA to ensure consistent event order
    esp_wifi_disconnect();
    esp_wifi_stop();
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta));
    ESP_ERROR_CHECK(esp_wifi_start());
    // Ensure STA runs with PS off for stability after applying new creds
    esp_wifi_set_ps(WIFI_PS_NONE);
    if (switched_to_sta) *switched_to_sta = true;
    return ESP_OK;
}

typedef struct {
    char ssid[33];
    char pass[65];
    uint32_t delay_ms;
} apply_ctx_t;

static void apply_task(void *arg)
{
    apply_ctx_t *ctx = (apply_ctx_t *)arg;
    vTaskDelay(pdMS_TO_TICKS(ctx->delay_ms));
    bool sw = false;
    wifi_mgr_apply_provision(ctx->ssid, ctx->pass, &sw);
    free(ctx);
    vTaskDelete(NULL);
}

esp_err_t wifi_mgr_defer_apply_provision(const char *ssid, const char *pass, uint32_t delay_ms)
{
    if (!ssid || !*ssid) return ESP_ERR_INVALID_ARG;
    apply_ctx_t *ctx = malloc(sizeof(apply_ctx_t));
    if (!ctx) return ESP_ERR_NO_MEM;
    memset(ctx, 0, sizeof(*ctx));
    strncpy(ctx->ssid, ssid, sizeof(ctx->ssid)-1);
    strncpy(ctx->pass, pass ? pass : "", sizeof(ctx->pass)-1);
    ctx->delay_ms = delay_ms ? delay_ms : 1000;
    if (xTaskCreatePinnedToCore(apply_task, "wifi_apply", 4096, ctx, 5, NULL, tskNO_AFFINITY) != pdPASS) {
        free(ctx);
        return ESP_FAIL;
    }
    return ESP_OK;
}
