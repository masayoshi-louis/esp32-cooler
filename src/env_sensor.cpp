#include "env_sensor.h"
#include <DHT.h>

namespace env_sensor
{

double temperature = 0;
double humidity = 0;

DHT dht;
long long lastSampleTs = 0;

void setup(uint8_t pin)
{
    dht.setup(pin);
}

void loop()
{
    if (abs(millis() - lastSampleTs) > dht.getMinimumSamplingPeriod())
    {
        humidity = dht.getHumidity();
        temperature = dht.getTemperature();
        lastSampleTs = millis();
    }
}

} // namespace env_sensor
