#include <Arduino.h>
#include <U8g2lib.h>
#include <OneWire.h>
#include <SPI.h>
#include <PID_v1.h>
#include "config.h"
#include "temperatures.h"
#include "env_sensor.h"
#include "monitor.h"
#include "dc_buck.h"

#define COOLER_FAN_PWM_CH 0
#define HEAT_SINK_FAN_PWM_CH 1
#define PUMP_PWM_CH 2
#define POWER_MODULE_FAN_PWM_CH 3
#define PWM_RESOLUTION 8
#define FOUR_PIN_FAN_PWM_FREQ 25000

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
uint8_t pumpVoltageLv;
uint8_t heatSinkFanVoltageLv;
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

BuckConverter pumpPowerControl(PUMP_PWM_CH, PUMP_PWM_PIN);
BuckConverter heatSinkFanPowerControl(HEAT_SINK_FAN_PWM_CH, HEAT_SINK_FAN_PWM_PIN);

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
    coolerFanPWM = (uint8_t)constrain(max(-coolerFanOutput1, -coolerFanOutput2), 0, COOLER_FAN_MAX_DUTY);
}

void computeHeatSinkFanOutput()
{
    heatSinkPID.Compute();
    heatSinkFanVoltageLv = (uint8_t)constrain(-heatSinkFanOutput, 0, 255);
    withThreshold(&heatSinkFanVoltageLv, 75);
}

void computePumpOutput()
{
    hotSidePID.Compute();
    pumpVoltageLv = (uint8_t)constrain(-pumpOutput, 0, 255);
    withThreshold(&pumpVoltageLv, 75);
}

void computeTecOutput()
{
    coldSidePID.Compute();
    tecPwrLv = (uint8_t)constrain(-tecOutput, 0, 255);
    withThreshold(&tecPwrLv, 30);
    tecEnabled = tecPwrLv > 0;
}

void computePowerModuleFanOutput()
{
    powerModuleFanPID.Compute();
    powerModuleFanPWM = (uint8_t)constrain(-powerModuleFanOutput, 0, POWER_MODULE_FAN_MAX_DUTY);
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
        Serial.print("Error: ");
        Serial.println(errMsg.c_str());
        exit(-1);
    }
}

void printStatus(void *);

void setup()
{
    Serial.begin(115200);

    // sensor setup
    env_sensor::setup(DHT22_PIN);

    // PWM setup
    ledcSetup(COOLER_FAN_PWM_CH, FOUR_PIN_FAN_PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(COOLER_FAN_PWM_PIN, COOLER_FAN_PWM_CH);

    ledcSetup(POWER_MODULE_FAN_PWM_CH, FOUR_PIN_FAN_PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(POWER_MODULE_FAN_PWM_PIN, POWER_MODULE_FAN_PWM_CH);

    pumpPowerControl.setup();
    heatSinkFanPowerControl.setup();

    // SPI (AD5262)
    Serial.printf("MOSI=%d, MISO=%d, SCK=%d, SS=%d\n", MOSI, MISO, SCK, TEC_PWR_AD5262_SS_PIN);
    pinMode(TEC_PWR_AD5262_SS_PIN, OUTPUT);
    digitalWrite(TEC_PWR_AD5262_SS_PIN, 1);
    SPI.begin();
    SPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE0));

    // monitor
    Monitor::setup();
    monitor.pumpVoltageCb = [](float value) {
        pumpPowerControl.onCurrentVoltageChanged(value);
    };
    monitor.heatSinkFanVoltageCb = [](float value) {
        heatSinkFanPowerControl.onCurrentVoltageChanged(value);
    };

    delay(3000);
    Serial.println("Started");

    xTaskCreatePinnedToCore(
        printStatus,
        "print_status",
        500,
        NULL,
        1,
        NULL,
        1);

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
    pumpPowerControl.setVoltage(12.0 / 255 * pumpVoltageLv);
    heatSinkFanPowerControl.setVoltage(12.0 / 255 * heatSinkFanVoltageLv);
    ledcWrite(POWER_MODULE_FAN_PWM_CH, powerModuleFanPWM);

    monitor.poll();
}

void printStatus(void *pvParams)
{
    while (1)
    {
        delay(5000);
        Serial.println("---------- START STATUS ----------");
        Serial.printf("[ENV] temperature=%.1f humidity=%.1f\n", env_sensor::temperature, env_sensor::humidity);
        Serial.printf("[COOLER FAN] PWM=%d PID_T=%.2f PID_H=%.2f\n", coolerFanPWM, coolerFanOutput1, coolerFanOutput2);
        Serial.printf("[TEC] V1=%.2f V2=%.2f PID=%.2f AMP=%.2f POWER=%.2f T_SET=%.2f T_COLD=%.2f\n", monitor.tecVoltages[0], monitor.tecVoltages[1], tecOutput, monitor.tecCurrent, monitor.tecPower(), coldSideSetpoint, temperatureSensors.coldSide);
        Serial.printf("[PUMP] V=%.2f V_SET=%.2f PID=%.2f T_SET=%.2f T_HOT=%.2f\n", pumpPowerControl.voltageCurrent, pumpPowerControl.voltageSetpoint, pumpOutput, hotSideSetpoint, temperatureSensors.hotSide);
        Serial.printf("[HEATSINK] V=%.2f V_SET=%.2f PID=%.2f T_SET=%.2f T_AIR=%.2f T_WATER=%.2f\n", heatSinkFanPowerControl.voltageCurrent, heatSinkFanPowerControl.voltageSetpoint, heatSinkFanOutput, waterSetpoint, temperatureSensors.outsideAir, temperatureSensors.water);
        Serial.printf("[POWER MODULE] T=%.2f PWM=%d PID=%.2f\n", temperatureSensors.powerModule, powerModuleFanPWM, powerModuleFanOutput);
        Serial.println("---------- END STATUS ------------\n");
    }
}
