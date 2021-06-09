#include "WiFi.h"

#include <driver/gpio.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_netif.h>
#include <esp_wifi.h>
#include <esp_wifi_default.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/task.h>
#include <lwip/err.h>
#include <lwip/sys.h>
#include <sdkconfig.h>
#include <string.h>
#include <assert.h>

static const char *TAG = "WiFi";

static void on_wifi_start(WiFi *p, esp_event_base_t base, int32_t id, void *info) {
    assert(base == WIFI_EVENT);
    assert(id == WIFI_EVENT_STA_START);

    esp_wifi_connect();
    ESP_LOGI(TAG, "Connecting...");
}

static void on_got_ip(WiFi *p, esp_event_base_t base, int32_t id, ip_event_got_ip_t *info) {
    assert(base == IP_EVENT);
    assert(id == IP_EVENT_STA_GOT_IP);
    assert(p->netif == info->esp_netif);

    p->ipv4.addr = info->ip_info.ip.addr;
    ESP_LOGI(TAG, "Interface \"%s\" address: " IPSTR, esp_netif_get_desc(p->netif), IP2STR(&p->ipv4));

    p->online = true;
    if (p->state_cb) p->state_cb(p);
}

static void on_wifi_disconnect(WiFi *p, esp_event_base_t base, int32_t id, wifi_event_sta_disconnected_t *info) {
    assert(base == WIFI_EVENT);
    assert(id == WIFI_EVENT_STA_DISCONNECTED);

    ESP_LOGI(TAG, "Disconnected!, reason=%i", info->reason);

    p->online = false;
    if (p->state_cb) p->state_cb(p);
}

void WiFi::connect(const char *s, const char *p) {
    ssid = s;
    password = p;

    esp_wifi_init(&phy);
    
    esp_event_handler_instance_register(WIFI_EVENT, WIFI_EVENT_STA_START, (esp_event_handler_t)on_wifi_start, this, nullptr);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, (esp_event_handler_t)on_got_ip, this, nullptr);
    esp_event_handler_instance_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, (esp_event_handler_t)on_wifi_disconnect, this, nullptr);

    assert(strlen(ssid) < sizeof(sta.sta.ssid));
    assert(strlen(password) < sizeof(sta.sta.password));
    bzero(&sta, sizeof(sta));
    memcpy(sta.sta.ssid, ssid, strlen(ssid) + 1);
    memcpy(sta.sta.password, password, strlen(password) + 1);
    sta.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &sta);

    esp_wifi_start();
    ESP_LOGI(TAG, "Connecting to %s", sta.sta.ssid);
}

void WiFi::reconnect() { if (!online) esp_wifi_connect(); }

// TODO disconnect/destructor untested

void WiFi::disconnect() {
    ESP_LOGI(TAG, "Disconnecting");

    esp_event_handler_unregister(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, (esp_event_handler_t)on_wifi_disconnect);
    esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, (esp_event_handler_t)on_got_ip);

    if (esp_wifi_stop() == ESP_ERR_WIFI_NOT_INIT) return;

    esp_wifi_deinit();
    esp_wifi_clear_default_wifi_driver_and_handlers(netif);
    esp_netif_destroy(netif);
}
