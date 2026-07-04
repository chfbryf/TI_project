#ifndef _MOTOR_H
#define _MOTOR_H

#include "ti_msp_dl_config.h"
#include "pid.h"

void Motor_Reset_L(void);
void Motor_Reset_R(void);
void App_PWM_Set_L(float Duty);
void App_PWM_Set_R(float Duty);
void App_Motor_SetOmega_L(float Omega_L);
void App_Motor_SetOmega_R(float Omega_R);
void App_Motor_Init(void);
void App_Motor_Proc(uint8_t flag);

#endif
