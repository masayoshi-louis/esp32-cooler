#include "env_sensor.h"
#include <DHT.h>

namespace env_sensor
{

float temperature = 0;
float humidity = 0;

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

String getErr()
{
    if (dht.getStatus() == DHT::ERROR_NONE)
    {
        return String();
    }
    return String("DHT sensor failure");
}

} // namespace env_sensor
