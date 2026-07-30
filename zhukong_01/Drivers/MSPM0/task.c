/**
 * @file    task.c
 * @brief   任务管理实现
 *
 * 5个任务槽位，按键1切换，主循环调用 Task_Run() 执行当前任务。
 */

#include "task.h"

volatile uint8_t task_num = 1;   /* 默认任务1 */

/* ═══════════════════════════════════════════════════════════════
 *  任务函数（暂时空出，后续填充）
 * ═══════════════════════════════════════════════════════════════ */

static void Task_1(void)
{
    /* TODO: 任务1 */
}

static void Task_2(void)
{
    /* TODO: 任务2 */
}

static void Task_3(void)
{
    /* TODO: 任务3 */
}

static void Task_4(void)
{
    /* TODO: 任务4 */
}

static void Task_5(void)
{
    /* TODO: 任务5 */
}

/* ═══════════════════════════════════════════════════════════════
 *  公开接口
 * ═══════════════════════════════════════════════════════════════ */

void Task_Init(void)
{
    task_num = 1;
}

void Task_SetNum(uint8_t num)
{
    if (num >= 1 && num <= TASK_MAX) {
        task_num = num;
    }
}

void Task_Run(void)
{
    switch (task_num) {
    case 1: Task_1(); break;
    case 2: Task_2(); break;
    case 3: Task_3(); break;
    case 4: Task_4(); break;
    case 5: Task_5(); break;
    default: break;
    }
}
