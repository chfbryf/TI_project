#ifndef STEP_MOTOR_H
#define STEP_MOTOR_H

#include "ti_msp_dl_config.h"

void step_motor_Init(void);
void step_motor_start(uint8_t steeper_id);
void step_set_speed(uint16_t speed, uint8_t stepper_id);
void step_set_angle(uint8_t angle, uint8_t stepper_id);

#endif
