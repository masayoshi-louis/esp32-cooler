#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_err.h>

void printWifiStatus()
{
    if (WiFi.status() == WL_CONNECTED)
    {
        printf("[WIFI]         Connected to %s\n", WiFi.SSID().c_str());
    }
    else
    {
        printf("[WIFI]         Not connected\n");
    }
}

void setHostName(const char *name)
{
    ESP_ERROR_CHECK(tcpip_adapter_set_hostname(TCPIP_ADAPTER_IF_STA, name));
}
