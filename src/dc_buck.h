#include <Arduino.h>

#ifndef _DC_BUCK_H
#define _DC_BUCK_H

class BuckConverter
{
private:
  uint16_t pwmDuty;
  uint8_t pwmPin;
  uint8_t ledcCh;
  static void loopTask(void *);
  SemaphoreHandle_t sync;

public:
  volatile float voltageSetpoint;
  volatile float voltageCurrent;
  float maxVoltage;
  BuckConverter(uint8_t ch, uint8_t pin);
  void loop();
  void setup();
  void setVoltage(float value);
  void onCurrentVoltageChanged(float value);
  float dutyCycle();
};

#endif
