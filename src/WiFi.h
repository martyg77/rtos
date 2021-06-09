// WiFi client

// Coordinate LwIP and WiFi lower layers with startup and applications
// Ref. https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/wifi.html

// Leverages Event Loop Library
// Ref. https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/esp_event.html

#pragma once

#include <esp_netif.h>
#include <esp_wifi.h>

class WiFi {
  public:
    void connect(const char *ssid, const char *password);
    void reconnect() { esp_wifi_connect(); }
    void disconnect();

    bool online = false;
    void (*state_cb)(bool online) = nullptr; // Called on ESP event loop timeslice

    esp_ip4_addr_t ipv4 = {.addr = 0};

    const char *ssid = nullptr;
    const char *password = nullptr;

    esp_netif_t *netif = esp_netif_create_default_wifi_sta();
    wifi_init_config_t phy = WIFI_INIT_CONFIG_DEFAULT();
    wifi_config_t sta;
};
