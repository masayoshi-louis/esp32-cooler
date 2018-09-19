#include "dc_buck.h"

#define PWM_FREQ 180000
#define PWM_RESOLUTION 16
#define PWM_STEP 100

void buckConverterTaskHandler(void *pvParams);

BuckConverter::BuckConverter(uint8_t ch, uint8_t pin)
{
    this->ledcCh = ch;
    this->pwmPin = pin;
    pwmDuty = 0;
    voltageCurrent = -1;
    voltageSetpoint = 0;
}

void BuckConverter::setup()
{
    static uint8_t instanceCounter = 0;
    char name[100];
    sprintf(name, "buck_converter-%d", instanceCounter++);
    ledcSetup(ledcCh, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(pwmPin, ledcCh);
    ledcWrite(ledcCh, pwmDuty);
    
    xTaskCreate(buckConverterTaskHandler, /* pvTaskCode */
                name,                     /* pcName */
                1000,                     /* usStackDepth */
                this,                     /* pvParameters */
                2,                        /* uxPriority */
                NULL);                    /* pxCreatedTask */
}

void BuckConverter::onCurrentVoltageChanged(float value)
{
    voltageCurrent = value;
}

void BuckConverter::setVoltage(float value)
{
    voltageSetpoint = value;
}

void BuckConverter::loop()
{
    if (voltageCurrent < 0)
        return;

    int32_t newPwmDuty = pwmDuty;
    if (voltageCurrent > voltageSetpoint)
    {
        newPwmDuty -= PWM_STEP;
    }
    if (voltageCurrent < voltageSetpoint)
    {
        newPwmDuty += PWM_STEP;
    }
    newPwmDuty = constrain(newPwmDuty, 0, 65535);
    if (newPwmDuty != pwmDuty)
    {
        ledcWrite(ledcCh, (uint16_t)newPwmDuty);
    }
}

void buckConverterTaskHandler(void *pvParams)
{
    BuckConverter *bc = (BuckConverter *)pvParams;
    while (1)
    {
        delayMicroseconds(250);
        bc->loop();
    }
}
