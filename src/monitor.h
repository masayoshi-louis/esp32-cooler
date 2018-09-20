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
  bool hallSensorPoll();
  bool pumpAndHeatSinkFanVoltagePoll();
  static void pollTask(void *);

public:
  typedef void (*VoltageListener_t)(float);
  Monitor();
  float tecVoltages[2];
  float tecCurrents[2];
  float tecPower();
  void poll();
  VoltageListener_t heatSinkFanVoltageListener;
  VoltageListener_t pumpVoltageListener;

  void setup();
};
