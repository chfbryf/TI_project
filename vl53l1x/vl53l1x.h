#ifndef VL53L1X_H
#define VL53L1X_H

#include <stdint.h>
#include <stdbool.h>

int8_t  VL53L1X_Init(void *i2c_handle);
int8_t  VL53L1X_GetDistance(uint16_t *distance_mm);
void    VL53L1X_NotifyDataReady(void);
bool    VL53L1X_DataReady(void);

#endif
