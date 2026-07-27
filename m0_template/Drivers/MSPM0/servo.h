#ifndef __SERVO_H__
#define __SERVO_H__

#include <stdint.h>

/* 舵机通道 */
#define SERVO_CH0    0   /* C0: PB14, TIMA0_CCP0 */
#define SERVO_CH1    1   /* C1: PA3,  TIMA0_CCP1 */

void Servo_Init(void);
void Set_Servo_Angle(uint8_t ch, unsigned int angle);
unsigned int Get_Servo_Angle(uint8_t ch);

#endif /* __SERVO_H__ */
