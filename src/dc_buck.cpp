#include "dc_buck.h"

#define PWM_FREQ 180000
#define PWM_RESOLUTION 16
#define PWM_DUTY_MAX 65535
#define PWM_STEP 100

void buckConverterTask(void *pvParams);

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

    xTaskCreate(buckConverterTask,        /* pvTaskCode */
                name,                     /* pcName */
                configMINIMAL_STACK_SIZE, /* usStackDepth */
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
    newPwmDuty = constrain(newPwmDuty, 0, PWM_DUTY_MAX);
    if (newPwmDuty != pwmDuty)
    {
        ledcWrite(ledcCh, (uint16_t)newPwmDuty);
    }
}

float BuckConverter::dutyCycle()
{
    return (float)pwmDuty / PWM_DUTY_MAX;
}

void buckConverterTask(void *pvParams)
{
    BuckConverter *bc = (BuckConverter *)pvParams;
    while (1)
    {
        delayMicroseconds(250);
        bc->loop();
    }
}
