#include <Arduino.h>
#include <U8g2lib.h>
#include <OneWire.h>
#include "config.h"
#include "temperatures.h"
#include "env_sensor.h"

OneWire oneWireBus(ONEWIRE_PIN);
Temperatures temperature_sensors(&oneWireBus);

void setup()
{
    Serial.begin(115200);
    env_sensor::dht_setup(DHT22_PIN);
}

void loop()
{
    // collect inputs
    temperature_sensors.loop();
    env_sensor::dht_loop();

    // compute outputs

    // write outputs
}
