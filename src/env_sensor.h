#include <Arduino.h>

namespace env_sensor
{

extern double temperature;
extern double humidity;

void setup(uint8_t pin);
void loop();

} // namespace env_sensor
