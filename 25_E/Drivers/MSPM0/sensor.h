#ifndef __SENSOR_H
#define __SENSOR_H

#include "sys.h"

#define L2          DL_GPIO_readPins(Sensor_P1_PORT,Sensor_P1_PIN)
#define L1          DL_GPIO_readPins(Sensor_P2_PORT,Sensor_P2_PIN)
#define M          DL_GPIO_readPins(Sensor_P3_PORT,Sensor_P3_PIN)
#define R1          DL_GPIO_readPins(Sensor_P4_PORT,Sensor_P4_PIN)
#define R2          DL_GPIO_readPins(Sensor_P5_PORT,Sensor_P5_PIN)

void Get_error(void);
void xunji_Proc(void);
void Motor_PID_Reset(void);

#endif
