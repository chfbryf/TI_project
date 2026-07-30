/**
 * @file    task.c
 * @brief   任务管理模块 - 5个任务的实现框架
 */

#include "sys.h"

/* ---------- 全局变量定义 ---------- */
/* task_id 使用 key.task_id，定义在 sys.c 中 */

/* ---------- 任务名称表 ---------- */
static const char* task_names[TASK_COUNT] = {
    "TSK1",
    "TSK2",
    "TSK3",
    "TSK4",
    "TSK5",
};

/* ---------- API 实现 ---------- */

/**
 * @brief 获取任务名称
 */
const char* Task_GetName(uint8_t id)
{
    if (id < 1 || id > TASK_COUNT) return "NONE";
    return task_names[id - 1];
}

/**
 * @brief 任务模块初始化
 */
void Task_Init(void)
{
    /* task_id 已由 main.c 通过 key.task_id = 1 初始化 */
}

/**
 * @brief 每轮主循环调用，根据当前 task_id 执行不同任务
 */
void Task_Run(void)
{
    switch (key.task_id) {
    case TASK_1:
        /* TODO: 任务1 具体行为 */
        break;
    case TASK_2:
        /* TODO: 任务2 具体行为 */
        break;
    case TASK_3:
        /* TODO: 任务3 具体行为 */
        break;
    case TASK_4:
        /* TODO: 任务4 具体行为 */
        break;
    case TASK_5:
        /* TODO: 任务5 具体行为 */
        break;
    default:
        break;
    }
}
