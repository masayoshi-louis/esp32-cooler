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
    lastSampleTs = 0;
}

void Temperatures::loop()
{
    if (abs(millis() - lastSampleTs) > 2000)
    {
        sensors->requestTemperatures();
        water = sensors->getTempC(_WATER_TEMP_PROBE_ADDR);
        coldSide = sensors->getTempC(_COLD_SIDE_TEMP_PROBE_ADDR);
        hotSide = sensors->getTempC(_HOT_SIDE_TEMP_PROBE_ADDR);
        powerModule = sensors->getTempC(_POWER_MODULE_TEMP_PROBE_ADDR);
        outsideAir = sensors->getTempC(_OUTSIDE_AIR_TEMP_PROBE_ADDR);
        lastSampleTs = millis();
    }
}

String Temperatures::getErr()
{
    if (water == -127.00)
    {
        return String("Water sensor failure");
    }
    if (coldSide == -127.00)
    {
        return String("Cold side sensor failure");
    }
    if (hotSide == -127.00)
    {
        return String("Hot side sensor failure");
    }
    if (powerModule == -127.00)
    {
        return String("Power module sensor failure");
    }
    if (outsideAir == -127.00)
    {
        return String("Outside air sensor failure");
    }
    return String();
}
