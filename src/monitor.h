#include <Arduino.h>

class Monitor
{
private:
  typedef bool (Monitor::*adc_poll_fn)();
  uint8_t adcSlot;
  adc_poll_fn *adcPolls;
  bool adcRunning;
  float readVoltage(uint8_t pin, float fullScale = 1.1);
  bool tecVoltagePoll();
  bool powerSensorPoll();
  bool pumpAndHeatSinkFanVoltagePoll();

public:
  Monitor();
  float powerVoltage;
  float tecVoltages[2];
  float tecCurrent;
  float tecPower();
  void poll();
  void (*heatSinkFanVoltageCb)(uint8_t);
  void (*pumpVoltageCb)(uint8_t);

  static void setup();
};
