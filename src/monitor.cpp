#include "monitor.h"
#include "config.h"

#define ADC_SLOTS 3

Monitor::Monitor(void)
{
    adcSlot = 0;
    tecVoltages[0] = tecVoltages[1] = 0;
    tecCurrents[0] = tecCurrents[1] = 0;
    adcPolls = (AdcPollHandler_t *)calloc(sizeof(AdcPollHandler_t), ADC_SLOTS);
    adcPolls[0] = &Monitor::tecVoltagePoll;
    adcPolls[1] = &Monitor::hallSensorPoll;
    adcPolls[2] = &Monitor::pumpAndHeatSinkFanVoltagePoll;
    adcRunning = false;
    pumpVoltageListener = NULL;
    heatSinkFanVoltageListener = NULL;
}

void Monitor::poll()
{
    auto pFn = adcPolls[adcSlot];
    adcRunning = (*this.*pFn)();
    if (!adcRunning)
    {
        // move to the next
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
    }
    else if (!adcBusy(TEC_V_CH1_PIN) && !adcBusy(TEC_V_CH2_PIN))
    {
        tecVoltages[0] = readVoltage(TEC_V_CH1_PIN) * TEC_V_SCALE * TEC_V_CH1_CAL;
        tecVoltages[1] = readVoltage(TEC_V_CH2_PIN) * TEC_V_SCALE * TEC_V_CH2_CAL;
        return false;
    }
    return true;
}

bool Monitor::pumpAndHeatSinkFanVoltagePoll()
{
    if (!adcRunning)
    {
        adcStart(PUMP_V_PIN);
        adcStart(HEAT_SINK_FAN_V_PIN);
    }
    else if (!adcBusy(PUMP_V_PIN) && !adcBusy(HEAT_SINK_FAN_V_PIN))
    {
        if (pumpVoltageListener)
        {
            (*pumpVoltageListener)(readVoltage(PUMP_V_PIN) * PUMP_V_SCALE * PUMP_V_CAL);
        }
        if (heatSinkFanVoltageListener)
        {
            (*heatSinkFanVoltageListener)(readVoltage(HEAT_SINK_FAN_V_PIN) * HEAT_SINK_V_SCALE * HEAT_SINK_V_CAL);
        }
        return false;
    }
    return true;
}

bool Monitor::hallSensorPoll()
{
    if (!adcRunning)
    {
        adcStart(HALL_SENSOR_CH1_PIN);
        adcStart(HALL_SENSOR_CH2_PIN);
    }
    else if (!adcBusy(HALL_SENSOR_CH1_PIN) && !adcBusy(HALL_SENSOR_CH2_PIN))
    {
        auto v = readVoltage(HALL_SENSOR_CH1_PIN, 3.9) * HALL_V_CH1_CAL;
        tecCurrents[0] = (v - 3.3 / 2) / HALL_V_PER_AMP;
        v = readVoltage(HALL_SENSOR_CH2_PIN, 3.9) * HALL_V_CH2_CAL;
        tecCurrents[1] = (v - 3.3 / 2) / HALL_V_PER_AMP;
        return false;
    }
    return true;
}

float Monitor::tecPower()
{
    return tecVoltages[0] * tecCurrents[0] + tecVoltages[1] * tecCurrents[1];
}

float Monitor::readVoltage(uint8_t pin, float fullScale)
{
    return (float)adcEnd(pin) / 4095 * fullScale;
}

void Monitor::setup()
{
    analogSetPinAttenuation(TEC_V_CH1_PIN, ADC_0db);
    analogSetPinAttenuation(TEC_V_CH2_PIN, ADC_0db);
    analogSetPinAttenuation(PUMP_V_PIN, ADC_0db);
    analogSetPinAttenuation(HEAT_SINK_FAN_V_PIN, ADC_0db);
    analogSetPinAttenuation(HALL_SENSOR_CH1_PIN, ADC_11db);
    analogSetPinAttenuation(HALL_SENSOR_CH2_PIN, ADC_11db);
    analogSetWidth(12);
    adcAttachPin(TEC_V_CH1_PIN);
    adcAttachPin(TEC_V_CH2_PIN);
    adcAttachPin(PUMP_V_PIN);
    adcAttachPin(HEAT_SINK_FAN_V_PIN);
    adcAttachPin(HALL_SENSOR_CH1_PIN);
    adcAttachPin(HALL_SENSOR_CH2_PIN);

    xTaskCreate(Monitor::pollTask,        /* pvTaskCode */
                "monitor_poll",           /* pcName */
                configMINIMAL_STACK_SIZE, /* usStackDepth */
                this,                     /* pvParameters */
                2,                        /* uxPriority */
                NULL);                    /* pxCreatedTask */
}

void Monitor::pollTask(void *pvParams)
{
    Monitor *m = (Monitor *)pvParams;
    for (;;)
    {
        delay(1);
        m->poll();
    }
}
