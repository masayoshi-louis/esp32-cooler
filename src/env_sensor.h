#include <Arduino.h>

namespace env_sensor
{

extern float temperature;
extern float humidity;

void setup(uint8_t pin);
void loop();
String getErr();

} // namespace env_sensor
