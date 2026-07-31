/**
 * @file    task.c
 * @brief   任务管理模块 - 5个任务的实现框架
 *
 * 任务1: 钢珠先到 7cm 稳定, 再到 16cm 稳定
 */

#include "sys.h"
#include "vision_control.h"

/* ---------- 全局变量定义 ---------- */
/* task_id 使用 key.task_id，定义在 sys.c 中 */

/* ---------- 任务1 内部状态 ---------- */
static task1_stage_t t1_stage    = T1_STAGE_GOTO_7;
static uint8_t       t1_init     = 0;  /* 是否已设置第一个目标 */

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
    t1_stage = T1_STAGE_GOTO_7;
    t1_init  = 0;
}

/**
 * @brief 每轮主循环调用，根据当前 task_id 执行不同任务
 */
void Task_Run(void)
{
    switch (key.task_id) {
    case TASK_1: {
        switch (t1_stage) {

        /* ── 阶段0: 先到 7cm ── */
        case T1_STAGE_GOTO_7:
            if (!t1_init) {
                VisionControl_SetTarget(7.0f);
                t1_init = 1;
            }
            if (VisionControl_IsStable()) {
                t1_stage = T1_STAGE_GOTO_16;
            }
            break;

        /* ── 阶段1: 再到 16cm ── */
        case T1_STAGE_GOTO_16:
            VisionControl_SetTarget(16.0f);
            if (VisionControl_IsStable()) {
                t1_stage = T1_STAGE_DONE;
            }
            break;

        /* ── 阶段2: 完成 ── */
        case T1_STAGE_DONE:
        default:
            break;
        }
        break;
    }
    case TASK_2:
        break;
    case TASK_3:
        break;
    case TASK_4:
        break;
    case TASK_5:
        break;
    default:
        break;
    }
}
