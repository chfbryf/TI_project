/**
 * @file    task.h
 * @brief   任务管理模块 - 5个任务的定义与接口
 */

#ifndef TASK_H
#define TASK_H

#include "ti_msp_dl_config.h"

/* ---------- 任务 ID 枚举 ---------- */
typedef enum {
    TASK_1 = 1,
    TASK_2 = 2,
    TASK_3 = 3,
    TASK_4 = 4,
    TASK_5 = 5,
} task_id_t;

#define TASK_COUNT  5

/* ---------- API ---------- */
const char* Task_GetName(uint8_t id);   /* 获取任务名称 */
void        Task_Init(void);            /* 任务模块初始化 */
void        Task_Run(void);             /* 每轮主循环调用，执行当前任务 */

#endif /* TASK_H */
