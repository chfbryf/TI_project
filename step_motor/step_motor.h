#ifndef STEP_MOTOR_H
#define STEP_MOTOR_H

#include "ti_msp_dl_config.h"

/* 全局变量：剩余步数（供 gimbal_tracker 直接读写） */
extern uint32_t step_remain_1;
extern uint32_t step_remain_2;

void step_motor_Init(void);
void step_motor_start(uint8_t steeper_id);
void step_motor_stop(uint8_t stepper_id);
void step_set_speed(float speed, uint8_t stepper_id);
void step_set_angle(float angle, uint8_t stepper_id);
void step_motor_dir_set(uint8_t direction, uint8_t stepper_id);
void step_rotate_by(float degrees, uint8_t direction, uint8_t stepper_id);

#endif
