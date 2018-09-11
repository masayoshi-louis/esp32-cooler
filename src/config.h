#include <Arduino.h>

#ifndef _CONFIG_H
#define _CONFIG_H

#define DHT22_PIN 0

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

#endif
