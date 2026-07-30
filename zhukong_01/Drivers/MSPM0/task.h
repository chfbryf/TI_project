/**
 * @file    task.h
 * @brief   任务管理模块
 *
 * 按键1 切换任务号(1~5)，主循环根据任务号执行对应任务。
 */

#ifndef TASK_H
#define TASK_H

#include <stdint.h>

#define TASK_MAX   5      /* 任务总数 */

extern volatile uint8_t task_num;  /* 当前任务号 1~5 */

void Task_Init(void);
void Task_Run(void);
void Task_SetNum(uint8_t num);

#endif /* TASK_H */
