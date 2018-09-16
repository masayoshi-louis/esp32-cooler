#include <Arduino.h>

#ifndef _DC_BUCK_H
#define _DC_BUCK_H

class BuckConverter
{
  private:
    uint16_t pwmDuty;
    uint8_t pwmPin;
    volatile uint8_t voltageSetpoint;
    volatile uint8_t voltageCurrent;

  public:
    BuckConverter(uint8_t pin);
    void loop();
    void setup();
    void setVoltage(uint8_t value);
    void onCurrentVoltageChanged(uint8_t value);
};

#endif
