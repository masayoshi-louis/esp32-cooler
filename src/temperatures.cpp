#include "temperatures.h"
#include "config.h"

DeviceAddress _WATER_TEMP_PROBE_ADDR = WATER_TEMP_PROBE_ADDR;
DeviceAddress _COLD_SIDE_TEMP_PROBE_ADDR = COLD_SIDE_TEMP_PROBE_ADDR;
DeviceAddress _HOT_SIDE_TEMP_PROBE_ADDR = HOT_SIDE_TEMP_PROBE_ADDR;
DeviceAddress _POWER_MODULE_TEMP_PROBE_ADDR = POWER_MODULE_TEMP_PROBE_ADDR;
DeviceAddress _OUTSIDE_AIR_TEMP_PROBE_ADDR = OUTSIDE_AIR_TEMP_PROBE_ADDR;

Temperatures::Temperatures(OneWire *oneWire)
{
    this->sensors = new DallasTemperature(oneWire);
    water = 0;
    coldSide = 0;
    hotSide = 0;
    powerModule = 0;
    outsideAir = 0;
}

void Temperatures::loop()
{
    sensors->requestTemperatures();
    water = sensors->getTempC(_WATER_TEMP_PROBE_ADDR);
    coldSide = sensors->getTempC(_COLD_SIDE_TEMP_PROBE_ADDR);
    hotSide = sensors->getTempC(_HOT_SIDE_TEMP_PROBE_ADDR);
    powerModule = sensors->getTempC(_POWER_MODULE_TEMP_PROBE_ADDR);
    outsideAir = sensors->getTempC(_OUTSIDE_AIR_TEMP_PROBE_ADDR);
}

void Temperatures::sampleTask(void *pvParams)
{
    Temperatures *t = (Temperatures *)pvParams;
    for (;;)
    {
        t->loop();
        String *errMsg = t->getErr();
        if (errMsg)
        {
            ESP_LOGE("TEMPERATURES", "%s", errMsg->c_str());
            exit(-1);
        }
        delay(2000);
    }
}

void Temperatures::begin()
{
    xTaskCreate(Temperatures::sampleTask,
                "temperatures_sample",
                configMINIMAL_STACK_SIZE,
                this,
                2,
                NULL);
}

String *Temperatures::getErr()
{
    if (water == -127.00)
    {
        return new String("Water sensor failure");
    }
    if (coldSide == -127.00)
    {
        return new String("Cold side sensor failure");
    }
    if (hotSide == -127.00)
    {
        return new String("Hot side sensor failure");
    }
    if (powerModule == -127.00)
    {
        return new String("Power module sensor failure");
    }
    if (outsideAir == -127.00)
    {
        return new String("Outside air sensor failure");
    }
    return NULL;
}
