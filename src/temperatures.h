#include <Arduino.h>
#include <DallasTemperature.h>

#ifndef _TEMPERATURES_H
#define _TEMPERATURES_H

class Temperatures
{
private:
  DallasTemperature *sensors;
  long long lastSampleTs;

public:
  float water;
  float coldSide;
  float hotSide;
  float pwr_module;
  Temperatures(OneWire *oneWire);
  void loop();
};

#endif
