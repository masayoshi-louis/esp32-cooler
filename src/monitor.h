#include <Arduino.h>

class Monitor
{
  private:
    typedef bool (Monitor::*adc_poll_fn)();
    uint8_t adcSlot;
    adc_poll_fn *adcPolls;
    bool adcRunning;
    float readVoltage(uint8_t pin);
    bool tecVoltagePoll();
    bool hallSensorPoll();

  public:
    Monitor();
    float tecVoltages[2];
    void poll();

    static void setup();
};
