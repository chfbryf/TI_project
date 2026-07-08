#ifndef __SENSOR_H
#define __SENSOR_H

#include "sys.h"

#define L4          DL_GPIO_readPins(Sensor_P1_PORT,Sensor_P1_PIN)
#define L3          DL_GPIO_readPins(Sensor_P2_PORT,Sensor_P2_PIN)
#define L2          DL_GPIO_readPins(Sensor_P3_PORT,Sensor_P3_PIN)
#define L1          DL_GPIO_readPins(Sensor_P4_PORT,Sensor_P4_PIN)
#define R1          DL_GPIO_readPins(Sensor_P5_PORT,Sensor_P5_PIN)
#define R2          DL_GPIO_readPins(Sensor_P6_PORT,Sensor_P6_PIN)
#define R3          DL_GPIO_readPins(Sensor_P7_PORT,Sensor_P7_PIN)
#define R4          DL_GPIO_readPins(Sensor_P8_PORT,Sensor_P8_PIN)

void Get_error(void);
int32_t Error(void);

#endif
