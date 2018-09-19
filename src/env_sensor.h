#include <Arduino.h>

namespace env_sensor
{

extern double temperature;
extern double humidity;

void begin(uint8_t pin);

} // namespace env_sensor
