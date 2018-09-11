#include "env_sensor.h"
#include <DHT.h>

namespace env_sensor
{

float temperature = 0;
float humidity = 0;

DHT dht;
long long lastSampleTs = 0;

void dht_setup(uint8_t pin)
{
    dht.setup(pin);
}

void dht_loop()
{
    if (abs(millis() - lastSampleTs) > dht.getMinimumSamplingPeriod())
    {
        humidity = dht.getHumidity();
        temperature = dht.getTemperature();
        lastSampleTs = millis();
    }
}

} // namespace env_sensor
