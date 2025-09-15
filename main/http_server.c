#include <stdio.h>
#include <string.h>
#include <dirent.h>
#include <unistd.h>
#include <sys/stat.h>
#include "esp_log.h"
#include "esp_http_server.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "wifi_mgr.h"
#include "http_server.h"
#include <ctype.h>

static const char *TAG = "http_srv";

#define NVS_NS_WIFI   "wifi"
#define NVS_KEY_SSID  "ssid"
#define NVS_KEY_PASS  "pass"
#define NVS_NS_APP    "app"
#define NVS_KEY_API   "api_key"

#define HTTP_PORT         CONFIG_GEEK_HTTP_PORT

static void html_escape(char *dst, size_t dst_sz, const char *src)
{
    size_t di = 0;
    for (const unsigned char *p = (const unsigned char *)src; *p && di + 6 < dst_sz; ++p) {
        if (*p == '&')      di += snprintf(dst+di, dst_sz-di, "&amp;");
        else if (*p == '<') di += snprintf(dst+di, dst_sz-di, "&lt;");
        else if (*p == '>') di += snprintf(dst+di, dst_sz-di, "&gt;");
        else if (*p == '"')di += snprintf(dst+di, dst_sz-di, "&quot;");
        else dst[di++] = *p;
    }
    dst[di] = 0;
}

static esp_err_t root_get_handler(httpd_req_t *req)
{
    nvs_handle_t n;
    char ssid[33] = {0};
    char pass[65] = {0};
    char api_key[129] = {0};
    size_t len;
    bool has_wifi = false, has_api = false;

    if (nvs_open(NVS_NS_WIFI, NVS_READONLY, &n) == ESP_OK) {
        len = sizeof(ssid);
        if (nvs_get_str(n, NVS_KEY_SSID, ssid, &len) == ESP_OK && strlen(ssid) > 0) {
            len = sizeof(pass);
            if (nvs_get_str(n, NVS_KEY_PASS, pass, &len) == ESP_OK) {
                if (strlen(pass) >= 8 || strlen(pass) == 0) {
                    has_wifi = true;
                }
            }
        }
        nvs_close(n);
    }
    if (nvs_open(NVS_NS_APP, NVS_READONLY, &n) == ESP_OK) {
        len = sizeof(api_key);
        has_api = (nvs_get_str(n, NVS_KEY_API, api_key, &len) == ESP_OK);
        nvs_close(n);
    }

    httpd_resp_sendstr_chunk(req, "<html><head><meta charset='utf-8'><title>Geek Recorder</title></head><body>");
    httpd_resp_sendstr_chunk(req, "<h2>Geek Recorder</h2>");
    httpd_resp_sendstr_chunk(req, "<p>Wi-Fi: ");
    httpd_resp_sendstr_chunk(req, has_wifi ? "configured" : "not configured");
    httpd_resp_sendstr_chunk(req, "</p><p>API Key: ");
    httpd_resp_sendstr_chunk(req, has_api ? "configured" : "not configured");
    httpd_resp_sendstr_chunk(req, "</p><h3>Provision</h3>");
    httpd_resp_sendstr_chunk(req, "<form method='POST' action='/provision'>SSID: <input id='ssid' name='ssid' maxlength='32' list='ssidlist' required> ");
    httpd_resp_sendstr_chunk(req, "Password: <input name='pass' type='password' maxlength='64' minlength='8' placeholder='>=8 chars for WPA/WPA2'> ");
    httpd_resp_sendstr_chunk(req, "API Key: <input name='api' maxlength='128'>");
    httpd_resp_sendstr_chunk(req, "<button type='submit'>Save</button> <button type='button' id='scan'>Scan Wi-Fi</button></form>");
    httpd_resp_sendstr_chunk(req, "<datalist id='ssidlist'></datalist>");
    httpd_resp_sendstr_chunk(req, "<div style='margin:6px 0;color:#666;font-size:12px;'>Device supports 2.4&nbsp;GHz (channels 1–13) only.</div>");
    httpd_resp_sendstr_chunk(req, "<div><select id='ssidsel' size='8' style='width:100%;max-width:420px;'></select></div>");
    httpd_resp_sendstr_chunk(req, "<script>\nconst btn=document.getElementById('scan');\nconst list=document.getElementById('ssidlist');\nconst sel=document.getElementById('ssidsel');\nconst ssid=document.getElementById('ssid');\nfunction authText(a){switch(a){case 0:return 'OPEN';case 1:return 'WEP';case 2:return 'WPA_PSK';case 3:return 'WPA2_PSK';case 4:return 'WPA_WPA2';case 5:return 'WPA2_ENT';case 6:return 'WPA3_PSK';case 7:return 'WPA2_WPA3';case 8:return 'OWE';default:return 'UNK';}}\nfunction fillLists(data){list.innerHTML='';sel.innerHTML='';let seen={};data.sort((a,b)=>b.rssi-a.rssi).forEach(ap=>{if(!ap||!ap.ssid||seen[ap.ssid])return;seen[ap.ssid]=1;const o=document.createElement('option');o.value=ap.ssid;list.appendChild(o);const opt=document.createElement('option');const ch=(ap.chan!=null?ap.chan:'?');opt.textContent=ap.ssid+'  (ch '+ch+', '+authText(ap.auth)+', '+ap.rssi+' dBm)';opt.value=ap.ssid;sel.appendChild(opt);});}\nsel.addEventListener('change',()=>{if(sel.value)ssid.value=sel.value;});\nasync function pollResult(maxTry=30){for(let i=0;i<maxTry;i++){let r=await fetch('/scan_result',{cache:'no-store'});if(!r.ok)throw new Error('result failed');let data=await r.json();if(Array.isArray(data)){return data;}if(data&&data.status!=='pending'){break;}await new Promise(res=>setTimeout(res,300));}throw new Error('timeout');}\nbtn&&btn.addEventListener('click',async()=>{btn.disabled=true;btn.textContent='Scanning...';try{const s=await fetch('/scan_start',{cache:'no-store'});if(!s.ok)throw new Error('start failed');const data=await pollResult();fillLists(data);if(data.length===0)alert('No AP found');}catch(e){alert('Scan failed');}finally{btn.disabled=false;btn.textContent='Scan Wi-Fi';}});\n</script>");
    httpd_resp_sendstr_chunk(req, "<h3>Files</h3><ul>");
    DIR *d = opendir("/sdcard");
    if (d) {
        struct dirent *de;
        while ((de = readdir(d)) != NULL) {
            if (de->d_name[0] == '.') continue;
            char esc[256]; html_escape(esc, sizeof(esc), de->d_name);
            httpd_resp_sendstr_chunk(req, "<li><a href='/download?f=");
            httpd_resp_sendstr_chunk(req, esc);
            httpd_resp_sendstr_chunk(req, "'>");
            httpd_resp_sendstr_chunk(req, esc);
            httpd_resp_sendstr_chunk(req, "</a> <a href='/delete?f=");
            httpd_resp_sendstr_chunk(req, esc);
            httpd_resp_sendstr_chunk(req, "' onclick=\"return confirm('Delete?')\">[delete]</a></li>");
        }
        closedir(d);
    } else {
        httpd_resp_sendstr_chunk(req, "<li>(SD not mounted)</li>");
    }
    httpd_resp_sendstr_chunk(req, "</ul></body></html>");
    return httpd_resp_send_chunk(req, NULL, 0);
}

static int httpd_recv_all(httpd_req_t *req, char *buf, size_t buf_sz)
{
    int total = 0;
    while (total < (int)buf_sz - 1 && total < req->content_len) {
        int r = httpd_req_recv(req, buf + total, buf_sz - 1 - total);
        if (r <= 0) break;
        total += r;
    }
    buf[total] = 0;
    return total;
}

static bool kv_get_str(const char *body, const char *key, char *out, size_t out_sz)
{
    if (!body || !key || !out || out_sz == 0) return false;
    size_t klen = strlen(key);
    const char *p = body;
    while (p && *p) {
        const char *amp = strchr(p, '&');
        const char *seg_end = amp ? amp : p + strlen(p);
        if ((size_t)(seg_end - p) >= klen + 1 && strncmp(p, key, klen) == 0 && p[klen] == '=') {
            const char *v = p + klen + 1;
            // Copy and URL-decode into out
            size_t oi = 0;
            for (const char *s = v; s < seg_end && oi + 1 < out_sz; ++s) {
                if (*s == '+') { out[oi++] = ' '; }
                else if (*s == '%' && s + 2 < seg_end && isxdigit((unsigned char)s[1]) && isxdigit((unsigned char)s[2])) {
                    int hi = (s[1] >= 'A' ? ((s[1] & ~0x20) - 'A' + 10) : (s[1] - '0'));
                    int lo = (s[2] >= 'A' ? ((s[2] & ~0x20) - 'A' + 10) : (s[2] - '0'));
                    out[oi++] = (char)((hi << 4) | lo);
                    s += 2;
                } else {
                    out[oi++] = *s;
                }
            }
            out[oi] = 0;
            return true;
        }
        if (!amp) break; else p = amp + 1;
    }
    if (out_sz) out[0] = 0;
    return false;
}

static esp_err_t provision_post_handler(httpd_req_t *req)
{
    char body[512];
    if (httpd_recv_all(req, body, sizeof body) <= 0) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad form");
    }
    char ssid[33] = {0};
    char pass[65] = {0};
    char api[129]  = {0};
    kv_get_str(body, "ssid", ssid, sizeof(ssid));
    kv_get_str(body, "pass", pass, sizeof(pass));
    kv_get_str(body, "api",  api,  sizeof(api));

    if (strlen(ssid) > 0) {
        if (strlen(pass) > 0 && strlen(pass) < 8) {
            ESP_LOGW(TAG, "Provision rejected: password too short (%d)", (int)strlen(pass));
            httpd_resp_set_type(req, "text/html; charset=utf-8");
            httpd_resp_set_status(req, "400 Bad Request");
            return httpd_resp_sendstr(req, "<html><body><p>Password must be at least 8 characters for WPA/WPA2 networks.</p><p><a href='/'>&larr; Back</a></p></body></html>");
        }
        // Save API key regardless
        nvs_handle_t n;
        if (nvs_open(NVS_NS_APP, NVS_READWRITE, &n) == ESP_OK) {
            nvs_set_str(n, NVS_KEY_API, api);
            nvs_commit(n);
            nvs_close(n);
        }
        // 延迟切换，确保HTTP响应完整送达后再关闭AP/切STA
        esp_err_t er = wifi_mgr_defer_apply_provision(ssid, pass, 1500);
        if (er != ESP_OK) {
            return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "apply failed");
        }
        // 返回确认页（避免重定向过程中AP关闭导致失败）
        httpd_resp_set_type(req, "text/html; charset=utf-8");
        return httpd_resp_sendstr(req, "<html><body><p>Saved. Device will connect to Wi‑Fi shortly...</p></body></html>");
    }
    return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "ssid required");
}

static void send_scan_json(httpd_req_t *req, const wifi_ap_record_t *recs, uint16_t ap_num)
{
    httpd_resp_set_type(req, "application/json; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_sendstr_chunk(req, "[");
    for (uint16_t i = 0; i < ap_num; ++i) {
        char ssid_esc[66] = {0};
        size_t di = 0;
        for (int j = 0; j < 32 && recs[i].ssid[j]; ++j) {
            unsigned char c = recs[i].ssid[j];
            if (c == '"' || c == '\\') {
                if (di + 2 >= sizeof(ssid_esc)) break;
                ssid_esc[di++] = '\\'; ssid_esc[di++] = c;
            } else if (c >= 0x20 && c < 0x7F) {
                if (di + 1 >= sizeof(ssid_esc)) break;
                ssid_esc[di++] = c;
            } else {
                if (di + 1 >= sizeof(ssid_esc)) break;
                ssid_esc[di++] = '?';
            }
        }
        char item[200];
        int n = snprintf(item, sizeof(item),
                         "%s{\"ssid\":\"%s\",\"rssi\":%d,\"auth\":%d,\"chan\":%u}",
                         (i ? "," : ""), ssid_esc, recs[i].rssi, recs[i].authmode, recs[i].primary);
        if (n > 0 && n < (int)sizeof(item)) httpd_resp_sendstr_chunk(req, item);
    }
    httpd_resp_sendstr_chunk(req, "]");
    httpd_resp_send_chunk(req, NULL, 0);
}

static esp_err_t scan_start_get_handler(httpd_req_t *req)
{
    esp_err_t err = wifi_mgr_scan_start();
    httpd_resp_set_type(req, "application/json");
    if (err == ESP_OK) {
        return httpd_resp_sendstr(req, "{\"status\":\"started\"}");
    } else {
        char msg[96];
        snprintf(msg, sizeof msg, "{\"status\":\"error\",\"code\":%d}", err);
        return httpd_resp_sendstr(req, msg);
    }
}

static esp_err_t scan_result_get_handler(httpd_req_t *req)
{
    bool pending; const wifi_ap_record_t *recs; uint16_t count;
    wifi_mgr_scan_status(&pending, &recs, &count);
    if (pending) {
        httpd_resp_set_type(req, "application/json");
        return httpd_resp_sendstr(req, "{\"status\":\"pending\"}");
    }
    if (recs && count) {
        send_scan_json(req, recs, count);
        return ESP_OK;
    }
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "[]");
}

static esp_err_t download_get_handler(httpd_req_t *req)
{
    char query[256]; char f[192];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "no query");
    if (httpd_query_key_value(query, "f", f, sizeof(f)) != ESP_OK) return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "no f");
    char full[256]; snprintf(full, sizeof(full), "/sdcard/%s", f);
    FILE *fp = fopen(full, "rb");
    if (!fp) return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "not found");
    httpd_resp_set_type(req, "application/octet-stream");
    httpd_resp_set_hdr(req, "Content-Disposition", "attachment");
    char buf[1024]; size_t n;
    while ((n = fread(buf, 1, sizeof(buf), fp)) > 0) {
        if (httpd_resp_send_chunk(req, buf, n) != ESP_OK) break;
    }
    fclose(fp);
    return httpd_resp_send_chunk(req, NULL, 0);
}

static esp_err_t delete_get_handler(httpd_req_t *req)
{
    char query[256]; char f[192];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "no query");
    if (httpd_query_key_value(query, "f", f, sizeof(f)) != ESP_OK) return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "no f");
    char full[256]; snprintf(full, sizeof(full), "/sdcard/%s", f);
    unlink(full);
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "/");
    return httpd_resp_send(req, NULL, 0);
}

static httpd_handle_t s_server = NULL;

esp_err_t http_server_start(httpd_handle_t *out)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port = HTTP_PORT;
    cfg.uri_match_fn = httpd_uri_match_wildcard;
    httpd_handle_t server = NULL;
    ESP_ERROR_CHECK(httpd_start(&server, &cfg));

    httpd_uri_t root = { .uri = "/", .method = HTTP_GET, .handler = root_get_handler, .user_ctx = NULL };
    httpd_uri_t provision = { .uri = "/provision", .method = HTTP_POST, .handler = provision_post_handler, .user_ctx = NULL };
    httpd_uri_t scan_start = { .uri = "/scan_start", .method = HTTP_GET, .handler = scan_start_get_handler, .user_ctx = NULL };
    httpd_uri_t scan_result = { .uri = "/scan_result", .method = HTTP_GET, .handler = scan_result_get_handler, .user_ctx = NULL };
    httpd_uri_t download = { .uri = "/download", .method = HTTP_GET, .handler = download_get_handler, .user_ctx = NULL };
    httpd_uri_t del = { .uri = "/delete", .method = HTTP_GET, .handler = delete_get_handler, .user_ctx = NULL };
    httpd_register_uri_handler(server, &root);
    httpd_register_uri_handler(server, &provision);
    httpd_register_uri_handler(server, &scan_start);
    httpd_register_uri_handler(server, &scan_result);
    httpd_register_uri_handler(server, &download);
    httpd_register_uri_handler(server, &del);
    s_server = server;
    if (out) *out = server;
    return ESP_OK;
}
