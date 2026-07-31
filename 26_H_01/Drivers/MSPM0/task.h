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

/* 任务1 阶段（小车循迹） */
typedef enum {
    T1_STAGE_IDLE     = 0,   /* 等待按键2启动 */
    T1_STAGE_TRACKING = 1,   /* 正在循迹 */
    T1_STAGE_STOPPED  = 2,   /* 已停车 */
} task1_stage_t;

/* 任务2 阶段 */
typedef enum {
    T2_STAGE_IDLE    = 0,   /* 等待按键2启动 */
    T2_STAGE_GOTO_7  = 1,   /* 先到 7cm */
    T2_STAGE_GOTO_16 = 2,   /* 再到 16cm */
    T2_STAGE_DONE    = 3,   /* 完成 */
} task2_stage_t;

/* 任务3 阶段（循迹 + 视觉推杆12cm） */
typedef enum {
    T3_STAGE_IDLE     = 0,   /* 等待按键2启动 */
    T3_STAGE_TRACKING = 1,   /* 正在循迹 + 钢珠12cm */
    T3_STAGE_STOPPED  = 2,   /* 已停车 */
} task3_stage_t;

/* 任务4 阶段（循迹 + 视觉自动标定） */
typedef enum {
    T4_STAGE_IDLE     = 0,   /* 等待按键2启动 */
    T4_STAGE_TRACKING = 1,   /* 正在循迹 + 视觉自动标定 */
    T4_STAGE_STOPPED  = 2,   /* 已停车 */
} task4_stage_t;

#define TASK_COUNT  5

/* ---------- API ---------- */
const char* Task_GetName(uint8_t id);   /* 获取任务名称 */
void        Task_Init(void);            /* 任务模块初始化 */
void        Task_Run(void);             /* 每轮主循环调用，执行当前任务 */
bool        Task_IsVisionActive(void);  /* 视觉推杆是否应激活（非IDLE） */

#endif /* TASK_H */
