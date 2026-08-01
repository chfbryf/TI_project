#ifndef SYS_H_
#define SYS_H_

#include "ti_msp_dl_config.h"
#include "main.h"
#include <stdio.h>
#include "key.h"
#include <stdint.h>
#include "ir_tracking.h"
#include "sensor2.h"
#include <math.h>
#include "step_motor.h"
#include "step_track.h"
#include "vision.h"
#include "vision_control.h"


#include "task.h"

/* 编译模式切换：0=从机  1=主机 */
#define HOST_MODE  0


#define LED3_High        DL_GPIO_setPins(LED_PORT, LED_LED3_PIN)
#define LED3_Low         DL_GPIO_clearPins(LED_PORT, LED_LED3_PIN)
#define LED4_High        DL_GPIO_setPins(LED_PORT, LED_LED4_PIN)
#define LED4_Low         DL_GPIO_clearPins(LED_PORT, LED_LED4_PIN)

#define BUZZ_High        DL_GPIO_setPins(BUZZER_PORT, BUZZER_buzzer_PIN)
#define BUZZ_Low         DL_GPIO_clearPins(BUZZER_PORT, BUZZER_buzzer_PIN)

/* 跨模块全局变量声明 */

extern key_t key;

extern volatile int16_t  base_speed;
extern volatile uint8_t  quanshu;
extern volatile uint32_t delay_flag;
extern volatile unsigned char Digtal;

/* 循迹任务计时变量（main.c 定义，task.c 使用） */
extern volatile uint32_t track_start_ms;
extern uint8_t  track_stopped;
extern float    track_final_sec;
extern uint8_t  decelerating;       /* 减速中标志 */

void speed(void);


#endif
