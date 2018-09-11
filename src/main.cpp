#include <Arduino.h>
#include <U8g2lib.h>
#include <OneWire.h>
#include <SPI.h>
#include <PID_v1.h>
#include "config.h"
#include "temperatures.h"
#include "env_sensor.h"

#define COOLER_FAN_PWM_CH 0
#define HEAT_SINK_FAN_PWM_CH 1
#define PUMP_PWM_CH 2
#define POWER_MODULE_FAN_PWM_CH 3
#define PWM_FREQ 22000
#define PWM_RESOLUTION 8

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
uint8_t pumpPWM;
uint8_t heatSinkFanPWM;
uint8_t powerModuleFanPWM;

// PID
PID envTemperaturePID(&env_sensor::temperature, &coolerFanOutput1, &temperatureSetpoint, 2, 5, 1, DIRECT);
PID envHumidityPID(&env_sensor::humidity, &coolerFanOutput2, &humiditySetpoint, 2, 5, 1, DIRECT);
PID coldSidePID(&temperatureSensors.coldSide, &tecOutput, &coldSideSetpoint, 2, 5, 1, DIRECT);
PID hotSidePID(&temperatureSensors.hotSide, &pumpOutput, &hotSideSetpoint, 2, 5, 1, DIRECT);
PID heatSinkPID(&temperatureSensors.water, &heatSinkFanOutput, &waterSetpoint, 2, 5, 1, DIRECT);
PID powerModuleFanPID(&temperatureSensors.powerModule, &powerModuleFanOutput, &powerModuleTemperatureSetpoint, 2, 5, 1, DIRECT);

uint8_t to256steps(double x)
{
    return (uint8_t)max(0, min(255, x));
}

void computeCoolerFanOutput()
{
    envTemperaturePID.Compute();
    envHumidityPID.Compute();
    coolerFanPWM = to256steps(max(-coolerFanOutput1, -coolerFanOutput2));
}

void computeHeatSinkFanOutput()
{
    heatSinkPID.Compute();
    heatSinkFanPWM = to256steps(-heatSinkFanPWM);
}

void computePumpOutput()
{
    hotSidePID.Compute();
    pumpPWM = to256steps(-pumpOutput);
}

void computeTecOutput()
{
    coldSidePID.Compute();
    tecPwrLv = to256steps(-tecOutput);
}

void computePowerModuleFanOutput()
{
    powerModuleFanPID.Compute();
    powerModuleFanPWM = to256steps(-powerModuleFanOutput);
}

void writeTecPwr()
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
    digitalWrite(TEC_ENABLE_PIN, tecPwrLv > 0);
}

void checkErr()
{
    if (errMsg.length() > 0)
    {
        exit(-1);
    }
}

void setup()
{
    Serial.begin(115200);

    // sensor setup
    env_sensor::setup(DHT22_PIN);

    // PWM setup
    ledcSetup(COOLER_FAN_PWM_CH, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(COOLER_FAN_PWM_PIN, COOLER_FAN_PWM_CH);

    ledcSetup(HEAT_SINK_FAN_PWM_CH, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(HEAT_SINK_FAN_PWM_PIN, HEAT_SINK_FAN_PWM_CH);

    ledcSetup(PUMP_PWM_CH, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(PUMP_PWM_PIN, PUMP_PWM_CH);

    ledcSetup(POWER_MODULE_FAN_PWM_CH, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(POWER_MODULE_FAN_PWM_PIN, POWER_MODULE_FAN_PWM_CH);

    // SPI (AD5262)
    Serial.printf("MOSI=%d, MISO=%d, SCK=%d, SS=%d\n", MOSI, MISO, SCK, TEC_PWR_AD5262_SS_PIN);
    pinMode(TEC_PWR_AD5262_SS_PIN, OUTPUT);
    digitalWrite(TEC_PWR_AD5262_SS_PIN, 1);
    SPI.begin();
    SPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE0));

    delay(3000);
    Serial.println("Started");
}

void loop()
{
    // collect inputs
    temperatureSensors.loop();
    errMsg = temperatureSensors.getErr();
    checkErr();

    env_sensor::loop();
    errMsg = env_sensor::getErr();
    checkErr();

    // adjust setpoints
    waterSetpoint = min(MAX_WATER_TEMPERATURE, temperatureSensors.outsideAir + SETPOINT_DELTA_T);
    hotSideSetpoint = temperatureSensors.water;
    coldSideSetpoint = max(10, env_sensor::temperature - 10);

    // compute outputs
    computeCoolerFanOutput();
    computeTecOutput();
    computePumpOutput();
    computeHeatSinkFanOutput();

    // write outputs
    ledcWrite(COOLER_FAN_PWM_CH, coolerFanPWM);
    writeTecPwr();
    ledcWrite(PUMP_PWM_CH, pumpPWM);
    ledcWrite(HEAT_SINK_FAN_PWM_CH, heatSinkFanPWM);
    ledcWrite(POWER_MODULE_FAN_PWM_CH, powerModuleFanPWM);
}
