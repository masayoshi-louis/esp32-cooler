#include <Arduino.h>

#ifndef _CONFIG_H
#define _CONFIG_H

#define CONFIG_ENABLE_CLI
#define CONFIG_HOSTNAME "esp32-cooler"

#define SETPOINT_T 30
#define SETPOINT_H 35
#define SETPOINT_DELTA_T 10
#define MAX_WATER_TEMPERATURE 45

// IO Pins

#define DHT22_PIN 12
#define COOLER_FAN_PWM_PIN 2
#define HEAT_SINK_FAN_PWM_PIN 26
#define PUMP_PWM_PIN 13
#define POWER_MODULE_FAN_PWM_PIN 17
#define TEC_V_CH1_PIN 33       // ADC1_CH5
#define TEC_V_CH2_PIN 14       // ADC2_CH6
#define PUMP_V_PIN 36          // ADC1_CH0
#define HEAT_SINK_FAN_V_PIN 25 // ADC2_CH8
#define HALL_SENSOR_CH1_PIN 39 // ADC1_CH3
#define HALL_SENSOR_CH2_PIN 15 // ADC2_CH3
#define TEC_ENABLE_PIN 4
#define ONEWIRE_PIN 16

// Dallas Temperatures

#define WATER_TEMP_PROBE_ADDR                          \
    {                                                  \
        0x28, 0x8D, 0x7B, 0x4D, 0x0A, 0x00, 0x00, 0xB4 \
    }
#define COLD_SIDE_TEMP_PROBE_ADDR                      \
    {                                                  \
        0x28, 0x74, 0x56, 0x73, 0x0A, 0x00, 0x00, 0x19 \
    }
#define HOT_SIDE_TEMP_PROBE_ADDR                       \
    {                                                  \
        0x28, 0x35, 0x04, 0x72, 0x0A, 0x00, 0x00, 0xC2 \
    }
#define POWER_MODULE_TEMP_PROBE_ADDR                   \
    {                                                  \
        0x28, 0x1A, 0xB4, 0x77, 0x91, 0x15, 0x02, 0xD8 \
    }

#define OUTSIDE_AIR_TEMP_PROBE_ADDR                    \
    {                                                  \
        0x28, 0x07, 0xAA, 0x4E, 0x0A, 0x00, 0x00, 0x8A \
    }

// TEC voltage sensor

#define TEC_V_R1 51000.0
#define TEC_V_R2 4700.0
#define TEC_V_SCALE ((TEC_V_R1 + TEC_V_R2) / (TEC_V_R2))
#define TEC_V_CH1_CAL 1.0
#define TEC_V_CH2_CAL 1.0

// Pump voltage sensor
#define PUMP_V_R1 51000.0
#define PUMP_V_R2 4700.0
#define PUMP_V_SCALE ((PUMP_V_R1 + PUMP_V_R2) / (PUMP_V_R2))
#define PUMP_V_CAL 1.0

// Heat sink fan voltage sensor
#define HEAT_SINK_V_R1 51000.0
#define HEAT_SINK_V_R2 4700.0
#define HEAT_SINK_V_SCALE ((HEAT_SINK_V_R1 + HEAT_SINK_V_R2) / (HEAT_SINK_V_R2))
#define HEAT_SINK_V_CAL 1.0

// TEC hall effect current sensor

#define HALL_V_PER_AMP 0.1
#define HALL_V_CH1_CAL 1.0
#define HALL_V_CH2_CAL 1.0

#endif
