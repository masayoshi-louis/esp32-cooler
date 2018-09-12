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
    if (!adcRunning)
    {
        adcStart(HALL_SENSOR_PIN);
        adcRunning = true;
        return false;
    }
    else
    {
        if (adcBusy(HALL_SENSOR_PIN))
        {
            return false;
        }
        auto v = readVoltage(HALL_SENSOR_PIN, 3.9);
        tecCurrent = (v - 3.3 / 2) / HALL_V_PER_AMP;
        adcRunning = false;
        return true;
    }
}

float Monitor::tecPower()
{
    return (tecVoltages[0] + tecVoltages[1]) / 2 * tecCurrent;
}

float Monitor::readVoltage(uint8_t pin, float fullScale)
{
    return (float)adcEnd(pin) / 4095 * fullScale;
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
