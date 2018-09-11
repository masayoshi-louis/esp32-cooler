#include "temperatures.h"
#include "config.h"

DeviceAddress _WATER_TEMP_PROBE_ADDR = WATER_TEMP_PROBE_ADDR;
DeviceAddress _COLD_SIDE_TEMP_PROBE_ADDR = COLD_SIDE_TEMP_PROBE_ADDR;
DeviceAddress _HOT_SIDE_TEMP_PROBE_ADDR = HOT_SIDE_TEMP_PROBE_ADDR;
DeviceAddress _POWER_MODULE_TEMP_PROBE_ADDR = POWER_MODULE_TEMP_PROBE_ADDR;

Temperatures::Temperatures(OneWire *oneWire)
{
    this->sensors = new DallasTemperature(oneWire);
    water = 0;
    coldSide = 0;
    hotSide = 0;
    pwr_module = 0;
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
        pwr_module = sensors->getTempC(_POWER_MODULE_TEMP_PROBE_ADDR);
        lastSampleTs = millis();
    }
}
