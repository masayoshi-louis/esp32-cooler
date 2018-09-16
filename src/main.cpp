#include <Arduino.h>
#include <U8g2lib.h>
#include <OneWire.h>
#include <SPI.h>
#include <PID_v1.h>
#include "config.h"
#include "temperatures.h"
#include "env_sensor.h"
#include "monitor.h"

#define COOLER_FAN_PWM_CH 0
#define HEAT_SINK_FAN_PWM_CH 1
#define PUMP_PWM_CH 2
#define POWER_MODULE_FAN_PWM_CH 3
#define PWM_RESOLUTION 8
#define FOUR_PIN_FAN_PWM_FREQ 25000
#define BUCK_CONVERTER_PWM_FREQ 180000

#define COOLER_FAN_MAX_DUTY 200
#define POWER_MODULE_FAN_MAX_DUTY 255

const uint8_t TEC_PWR_AD5262_SS_PIN = SS;

String errMsg;

OneWire oneWireBus(ONEWIRE_PIN);
Temperatures temperatureSensors(&oneWireBus);

// setpoints
double temperatureSetpoint = SETPOINT_T;
double humiditySetpoint = SETPOINT_H;
double coldSideSetpoint;
double hotSideSetpoint;
double waterSetpoint;
double powerModuleTemperatureSetpoint = 50;

// outputs
double coolerFanOutput1;
double coolerFanOutput2;
double tecOutput;
double pumpOutput;
double heatSinkFanOutput;
double powerModuleFanOutput;

uint8_t coolerFanPWM;
uint8_t tecPwrLv;
bool tecEnabled;
uint8_t pumpPWM;
uint8_t heatSinkFanPWM;
uint8_t powerModuleFanPWM;

// PID
PID envTemperaturePID(&env_sensor::temperature, &coolerFanOutput1, &temperatureSetpoint, 15, 10, 15, DIRECT);
PID envHumidityPID(&env_sensor::humidity, &coolerFanOutput2, &humiditySetpoint, 15, 10, 15, DIRECT);
PID coldSidePID(&temperatureSensors.coldSide, &tecOutput, &coldSideSetpoint, 15, 28, 10, DIRECT);
PID hotSidePID(&temperatureSensors.hotSide, &pumpOutput, &hotSideSetpoint, 50, 30, 15, DIRECT);
PID heatSinkPID(&temperatureSensors.water, &heatSinkFanOutput, &waterSetpoint, 15, 30, 10, DIRECT);
PID powerModuleFanPID(&temperatureSensors.powerModule, &powerModuleFanOutput, &powerModuleTemperatureSetpoint, 40, 30, 10, DIRECT);

// others
bool modeOn;
Monitor monitor;

uint8_t to256steps(double x)
{
    return (uint8_t)max(0, min(255, x));
}

void withThreshold(uint8_t *x, uint8_t t)
{
    if (*x < t)
    {
        *x = 0;
    }
}

void computeCoolerFanOutput()
{
    envTemperaturePID.Compute();
    envHumidityPID.Compute();
    coolerFanPWM = to256steps(max(-coolerFanOutput1, -coolerFanOutput2));
    coolerFanPWM = min(COOLER_FAN_MAX_DUTY, coolerFanPWM);
}

void computeHeatSinkFanOutput()
{
    heatSinkPID.Compute();
    heatSinkFanPWM = to256steps(-heatSinkFanPWM);
    withThreshold(&heatSinkFanPWM, 75);
}

void computePumpOutput()
{
    hotSidePID.Compute();
    pumpPWM = to256steps(-pumpOutput);
    withThreshold(&pumpPWM, 75);
}

void computeTecOutput()
{
    coldSidePID.Compute();
    tecPwrLv = to256steps(-tecOutput);
    withThreshold(&tecPwrLv, 30);
    tecEnabled = tecPwrLv > 0;
}

void computePowerModuleFanOutput()
{
    powerModuleFanPID.Compute();
    powerModuleFanPWM = to256steps(-powerModuleFanOutput);
    powerModuleFanPWM = min(POWER_MODULE_FAN_MAX_DUTY, powerModuleFanPWM);
}

void writeTec()
{
    static uint16_t lastValue = 65535;
    if (abs(lastValue - tecPwrLv) > 4)
    {
        digitalWrite(TEC_PWR_AD5262_SS_PIN, 0);
        SPI.transfer(0b00000001);
        SPI.transfer(tecPwrLv);
        digitalWrite(TEC_PWR_AD5262_SS_PIN, 1);
        delay(50);
        digitalWrite(TEC_PWR_AD5262_SS_PIN, 0);
        SPI.transfer(0b00000000);
        SPI.transfer(tecPwrLv);
        digitalWrite(TEC_PWR_AD5262_SS_PIN, 1);
        lastValue = tecPwrLv;
    }
    digitalWrite(TEC_ENABLE_PIN, modeOn && tecEnabled);
}

void checkErr()
{
    if (errMsg.length() > 0)
    {
        digitalWrite(TEC_ENABLE_PIN, LOW);
        exit(-1);
    }
}

float readVoltage(uint8_t pin)
{
    return (float)adcEnd(pin) / 1023 * 1.1 * TEC_V_SCALE;
}

void setup()
{
    Serial.begin(115200);

    // sensor setup
    env_sensor::setup(DHT22_PIN);

    // PWM setup
    ledcSetup(COOLER_FAN_PWM_CH, FOUR_PIN_FAN_PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(COOLER_FAN_PWM_PIN, COOLER_FAN_PWM_CH);

    ledcSetup(HEAT_SINK_FAN_PWM_CH, BUCK_CONVERTER_PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(HEAT_SINK_FAN_PWM_PIN, HEAT_SINK_FAN_PWM_CH);

    ledcSetup(PUMP_PWM_CH, BUCK_CONVERTER_PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(PUMP_PWM_PIN, PUMP_PWM_CH);

    ledcSetup(POWER_MODULE_FAN_PWM_CH, FOUR_PIN_FAN_PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(POWER_MODULE_FAN_PWM_PIN, POWER_MODULE_FAN_PWM_CH);

    // SPI (AD5262)
    Serial.printf("MOSI=%d, MISO=%d, SCK=%d, SS=%d\n", MOSI, MISO, SCK, TEC_PWR_AD5262_SS_PIN);
    pinMode(TEC_PWR_AD5262_SS_PIN, OUTPUT);
    digitalWrite(TEC_PWR_AD5262_SS_PIN, 1);
    SPI.begin();
    SPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE0));

    // monitor
    Monitor::setup();

    delay(3000);
    Serial.println("Started");

    modeOn = true;
}

void loop()
{
    // collect inputs
    monitor.poll();

    temperatureSensors.loop();
    errMsg = temperatureSensors.getErr();
    checkErr();

    env_sensor::loop();
    errMsg = env_sensor::getErr();
    checkErr();

    monitor.poll();

    // adjust setpoints
    waterSetpoint = min(MAX_WATER_TEMPERATURE, temperatureSensors.outsideAir + SETPOINT_DELTA_T);
    hotSideSetpoint = min(temperatureSensors.water, temperatureSensors.hotSide);
    coldSideSetpoint = max(10, env_sensor::temperature - 10);

    // compute outputs
    computeCoolerFanOutput();
    computeTecOutput();
    computePumpOutput();
    computeHeatSinkFanOutput();

    monitor.poll();

    // write outputs
    ledcWrite(COOLER_FAN_PWM_CH, coolerFanPWM);
    writeTec();
    ledcWrite(PUMP_PWM_CH, pumpPWM);
    ledcWrite(HEAT_SINK_FAN_PWM_CH, heatSinkFanPWM);
    ledcWrite(POWER_MODULE_FAN_PWM_CH, powerModuleFanPWM);

    monitor.poll();
}
