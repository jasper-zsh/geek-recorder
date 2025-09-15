#pragma once

#include "esp_err.h"
#include "esp_http_server.h"
#include "esp_wifi.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Initialize Wi-Fi (provisioning AP or STA based on NVS)
esp_err_t wifi_mgr_init(void);

// Return true if device is in provisioning mode (SoftAP active)
bool wifi_mgr_in_provisioning(void);

// Enter provisioning mode (start SoftAP) without clearing saved credentials.
// Keeps existing STA creds in NVS; switches Wi‑Fi to AP+STA for configuration.
esp_err_t wifi_mgr_enter_provisioning(void);

// Start async Wi-Fi scan (non-blocking)
esp_err_t wifi_mgr_scan_start(void);

// Get current scan status and snapshot of results pointer/count
// The returned pointer is owned by wifi_mgr and must not be freed by caller.
void wifi_mgr_scan_status(bool *pending, const wifi_ap_record_t **recs, uint16_t *count);

// Apply new Wi-Fi credentials and switch to STA-only immediately.
// Returns ESP_OK on success, and sets switched_to_sta=true if mode switched.
esp_err_t wifi_mgr_apply_provision(const char *ssid, const char *pass, bool *switched_to_sta);

// Schedule switching to STA-only after delay_ms, response-safe for HTTP flows.
// Credentials will be saved to NVS before switching as part of apply.
esp_err_t wifi_mgr_defer_apply_provision(const char *ssid, const char *pass, uint32_t delay_ms);

#ifdef __cplusplus
}
#endif
