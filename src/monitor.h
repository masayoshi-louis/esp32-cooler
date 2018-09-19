#include <Arduino.h>

class Monitor
{
private:
  typedef bool (Monitor::*AdcPollHandler_t)();
  uint8_t adcSlot;
  AdcPollHandler_t *adcPolls;
  bool adcRunning;
  float readVoltage(uint8_t pin, float fullScale = 1.1);
  bool tecVoltagePoll();
  bool powerSensorPoll();
  bool pumpAndHeatSinkFanVoltagePoll();

public:
  typedef void (*VoltageListener_t)(float);
  Monitor();
  float powerVoltage;
  float tecVoltages[2];
  float tecCurrent;
  float tecPower();
  void poll();
  VoltageListener_t heatSinkFanVoltageListener;
  VoltageListener_t pumpVoltageListener;

  void setup();
};
