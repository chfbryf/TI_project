/**
  ******************************************************************************
  * @file    pid.h
  * @author  铁头山羊stm32工作组
  * @version V1.0.0
  * @date    2024年10月27日
  * @brief   pid算法库
  ******************************************************************************
  * @attention
  * Copyright (c) 2024 -  东莞市明玉科技有限公司. 保留所有权力.
  ******************************************************************************
*/

#ifndef _PID_H_
#define _PID_H_

#include <stdint.h>
#include "lpf.h"

typedef struct{
	float Kp; // 比例环节系数 Kp
	float Ki; // 积分环节系数 Ki
	float Kd; // 微分环节系数 Kd
	float Setpoint; // PID的参考值
	float OutputUpperLimit; // 输出上限
	float OutputLowerLimit; // 输出下限
	float DefaultOutput; // 默认输出值
}PID_InitTypeDef;

typedef struct{
	PID_InitTypeDef Init;
	uint8_t cmd; // 0 - PID禁止 1 - PID使能
	uint64_t LastTime; // PID上次运行的时间，用于计算Δt
	float LastOutput;  // PID上次的输出
	float ITerm; // 积分项
	float DTerm; // 微分项
	float LastInput; // 上次的输入值
	float LastError; // 上次的误差值
	float Kp;
	float Ki;
	float Kd;
	float OutputUpperLimit;
	float OutputLowerLimit;
	float Setpoint;
	LPF_TypeDef Lpf; // 低通滤波器，该滤波器级联在PID的输出端
	uint8_t LpfCmd; // 低通滤波器开关，0-禁止，1-使能
}PID_TypeDef;

 void PID_Init(PID_TypeDef *PID, PID_InitTypeDef *PID_InitStruct);
 void PID_LpfConfig(PID_TypeDef *PID, float Tf, uint8_t NewState);
 void PID_Cmd(PID_TypeDef *PID, uint8_t NewState);
 void PID_Reset(PID_TypeDef *PID);
float PID_Compute1(PID_TypeDef *PID, float Input, uint64_t Now);
float PID_Compute2(PID_TypeDef *PID, float Input, float dInput, uint64_t Now);
 void PID_ChangeTunings(PID_TypeDef *PID, float NewKp, float NewKi, float NewKd);
 void PID_ChangeSetpoint(PID_TypeDef *PID, float NewSetpoint);

float PID_Compute(PID_TypeDef *PID, float Input);
void PID_Init_Simple(PID_TypeDef *PID, float Kp, float Ki, float Kd);
void PID_LimitConfig(PID_TypeDef *PID, float UpperLimit, float LowerLimit);
void PID_ChangeSP(PID_TypeDef *PID, float Setpoint);
void PID_Set_Params(PID_TypeDef *PID, float Kp, float Ki, float Kd);

#endif
