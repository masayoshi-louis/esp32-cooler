#include <Arduino.h>
#include <WiFi.h>

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
