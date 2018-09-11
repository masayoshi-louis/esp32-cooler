#include <Arduino.h>

namespace env_sensor
{

extern float temperature;
extern float humidity;

void dht_setup(uint8_t pin);
void dht_loop();

} // namespace env_sensor
