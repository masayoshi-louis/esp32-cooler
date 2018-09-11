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
  double water;
  double coldSide;
  double hotSide;
  double powerModule;
  double outsideAir; 
  Temperatures(OneWire *oneWire);
  void loop();
};

#endif
