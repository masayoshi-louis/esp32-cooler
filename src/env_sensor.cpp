#include "env_sensor.h"
#include <DHT.h>

namespace env_sensor
{

double temperature = 0;
double humidity = 0;

DHT dht;

static void sampleTask(void *pvParams)
{
    for (;;)
    {
        humidity = dht.getHumidity();
        temperature = dht.getTemperature();
        if (dht.getStatus() != DHT::ERROR_NONE)
        {
            ESP_LOGE("DHT", "Sensor failure, code=%d", dht.getStatus());
            exit(-1);
        }
        delay(dht.getMinimumSamplingPeriod() + 10);
    }
}

void begin(uint8_t pin)
{
    dht.setup(pin);
    xTaskCreate(sampleTask, "dht_sample", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
}

} // namespace env_sensor
