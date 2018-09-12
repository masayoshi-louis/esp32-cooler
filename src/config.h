#include <Arduino.h>

#ifndef _CONFIG_H
#define _CONFIG_H

#define SETPOINT_T 30
#define SETPOINT_H 35
#define SETPOINT_DELTA_T 10
#define MAX_WATER_TEMPERATURE 45

#define DHT22_PIN 0
#define COOLER_FAN_PWM_PIN 0
#define HEAT_SINK_FAN_PWM_PIN 0
#define PUMP_PWM_PIN 0
#define POWER_MODULE_FAN_PWM_PIN 0
#define TEC_V_CH1_PIN 35 // ADC1_CH7
#define TEC_V_CH2_PIN 25 // ADC2_CH8
#define HALL_SENSOR_PIN 26 // ADC2_CH9

#define TEC_ENABLE_PIN 0

#define ONEWIRE_PIN 0

#define WATER_TEMP_PROBE_ADDR                          \
    {                                                  \
        0x28, 0x8D, 0x7B, 0x4D, 0x0A, 0x00, 0x00, 0xB4 \
    }
#define COLD_SIDE_TEMP_PROBE_ADDR                      \
    {                                                  \
        0x28, 0x8D, 0x7B, 0x4D, 0x0A, 0x00, 0x00, 0xB4 \
    }
#define HOT_SIDE_TEMP_PROBE_ADDR                       \
    {                                                  \
        0x28, 0x8D, 0x7B, 0x4D, 0x0A, 0x00, 0x00, 0xB4 \
    }
#define POWER_MODULE_TEMP_PROBE_ADDR                   \
    {                                                  \
        0x28, 0x8D, 0x7B, 0x4D, 0x0A, 0x00, 0x00, 0xB4 \
    }

#define OUTSIDE_AIR_TEMP_PROBE_ADDR                    \
    {                                                  \
        0x28, 0x8D, 0x7B, 0x4D, 0x0A, 0x00, 0x00, 0xB4 \
    }

#define TEC_V_R1 51000.0
#define TEC_V_R2 4700.0
#define TEC_V_SCALE ((TEC_V_R1 + TEC_V_R2) / (TEC_V_R2))
#define TEC_V_CH1_CAL 1.0
#define TEC_V_CH2_CAL 1.0

#endif
