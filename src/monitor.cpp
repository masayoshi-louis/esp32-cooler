#include "monitor.h"
#include "config.h"

#define ADC_SLOTS 2

Monitor::Monitor(void)
{
    adcSlot = 0;
    tecVoltages[0] = 0;
    tecVoltages[1] = 0;
    adcPolls = (adc_poll_fn *)calloc(sizeof(adc_poll_fn), ADC_SLOTS);
    adcPolls[0] = &Monitor::tecVoltagePoll;
    adcPolls[1] = &Monitor::hallSensorPoll;
    adcRunning = false;
}

void Monitor::poll()
{
    auto pFn = adcPolls[adcSlot];
    if ((*this.*pFn)())
    {
        if (++adcSlot == ADC_SLOTS)
        {
            adcSlot = 0;
        }
    }
}

bool Monitor::tecVoltagePoll()
{
    if (!adcRunning)
    {
        adcStart(TEC_V_CH1_PIN);
        adcStart(TEC_V_CH2_PIN);
        adcRunning = true;
        return false;
    }
    else
    {
        if (adcBusy(TEC_V_CH1_PIN) || adcBusy(TEC_V_CH2_PIN))
        {
            return false;
        }
        tecVoltages[0] = readVoltage(TEC_V_CH1_PIN) * TEC_V_CH1_CAL;
        tecVoltages[1] = readVoltage(TEC_V_CH2_PIN) * TEC_V_CH2_CAL;
        adcRunning = false;
        return true;
    }
}

bool Monitor::hallSensorPoll()
{
    return true;
}

float Monitor::readVoltage(uint8_t pin)
{
    return (float)adcEnd(pin) / 4095 * 1.1 * TEC_V_SCALE;
}

void Monitor::setup()
{
    analogSetPinAttenuation(TEC_V_CH1_PIN, ADC_0db);
    analogSetPinAttenuation(TEC_V_CH2_PIN, ADC_0db);
    analogSetPinAttenuation(HALL_SENSOR_PIN, ADC_11db);
    analogSetWidth(12);
    adcAttachPin(TEC_V_CH1_PIN);
    adcAttachPin(TEC_V_CH2_PIN);
    adcAttachPin(HALL_SENSOR_PIN);
}
