/**
 * @file    task.c
 * @brief   任务管理模块 - 5个任务的实现框架
 *
 * 任务1: 小车循迹，按键2启动，十字停车，记录时间
 * 任务2: 钢珠先到 7cm 稳定, 再到 16cm 稳定（带稳定帧计数）
 * 任务3: 小车循迹 + 视觉推杆控制（钢珠保持12cm），十字停车，记录时间
 * 任务4: 小车循迹 + 视觉推杆控制（前3帧自动标定目标），十字停车，记录时间
 */

#include "sys.h"
#include "vision_control.h"

/* ---------- 全局变量定义 ---------- */
/* task_id 使用 key.task_id，定义在 sys.c 中 */

/* ---------- 任务1 内部状态（循迹） ---------- */
static task1_stage_t t1_stage = T1_STAGE_IDLE;

/* ---------- 任务2 内部状态 ---------- */
static task2_stage_t t2_stage      = T2_STAGE_IDLE;
static uint8_t       t2_init       = 0;   /* 是否已设置当前目标 */
static uint8_t       t2_stable_cnt = 0;   /* 稳定帧计数器 */

/* ---------- 任务3 内部状态（循迹 + 视觉推杆） ---------- */
static task3_stage_t t3_stage = T3_STAGE_IDLE;

/* ---------- 任务4 内部状态（循迹 + 视觉自动标定） ---------- */
static task4_stage_t t4_stage = T4_STAGE_IDLE;

/* ---------- 任务名称表 ---------- */
static const char* task_names[TASK_COUNT] = {
    "TRAC",
    "TSK2",
    "TRA2",
    "AUTO",
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
    t1_stage = T1_STAGE_IDLE;
    t2_stage      = T2_STAGE_IDLE;
    t2_init       = 0;
    t2_stable_cnt = 0;
    t3_stage      = T3_STAGE_IDLE;
    t4_stage      = T4_STAGE_IDLE;
}

/**
 * @brief 每轮主循环调用，根据当前 task_id 执行不同任务
 */
void Task_Run(void)
{
    static uint8_t prev_task_id = 0;

    /* 检测任务切换，重置新任务状态 */
    if (prev_task_id != key.task_id) {
        prev_task_id = key.task_id;
        if (key.task_id == 1) {
            t1_stage  = T1_STAGE_IDLE;
            key.start = 0;
        }
        if (key.task_id == 2) {
            t2_stage = T2_STAGE_IDLE;
        }
        if (key.task_id == 3) {
            t3_stage  = T3_STAGE_IDLE;
            key.start = 0;
        }
        if (key.task_id == 4) {
            t4_stage  = T4_STAGE_IDLE;
            key.start = 0;
        }
    }

    switch (key.task_id) {
    case TASK_1: {
        switch (t1_stage) {

        /* ── 阶段0: 等待按键2启动 ── */
        case T1_STAGE_IDLE:
            track_stopped = 0;              /* 清除上次停车标志 */
            if (key.keynum == 2) {
                key.start = 1;
                t1_stage  = T1_STAGE_TRACKING;
            }
            break;

        /* ── 阶段1: 正在循迹，等待停车 ── */
        case T1_STAGE_TRACKING:
            if (track_stopped) {
                t1_stage  = T1_STAGE_STOPPED;
            }
            break;

        /* ── 阶段2: 已停车，时间保留在OLED上 ── */
        case T1_STAGE_STOPPED:
        default:
            break;
        }
        break;
    }
    case TASK_2: {
        switch (t2_stage) {

        /* ── 阶段0: 等待按键2启动 ── */
        case T2_STAGE_IDLE: {
            static uint8_t k2_prev = 0;
            uint8_t k2_rising = (key.keynum == 2) && !k2_prev;
            k2_prev = (key.keynum == 2);
            if (k2_rising) {
                t2_init       = 0;
                t2_stable_cnt = 0;
                t2_stage      = T2_STAGE_GOTO_7;
            }
            break;
        }

        /* ── 阶段1: 先到 7cm ── */
        case T2_STAGE_GOTO_7:
            if (!t2_init) {
                VisionControl_SetTarget(6.0f);
                t2_init = 1;
            }
            if (VisionControl_IsStable()) {
                t2_stable_cnt++;
                if (t2_stable_cnt >= 3) {
                    t2_stable_cnt = 0;
                    t2_init       = 0;
                    t2_stage      = T2_STAGE_GOTO_16;
                }
            } else {
                t2_stable_cnt = 0;
            }
            break;

        /* ── 阶段2: 再到 16cm ── */
        case T2_STAGE_GOTO_16:
            if (!t2_init) {
                VisionControl_SetTarget(16.0f);
                t2_init = 1;
            }
            if (VisionControl_IsStable()) {
                t2_stable_cnt++;
                if (t2_stable_cnt >= 3) {
                    t2_stable_cnt = 0;
                    t2_stage      = T2_STAGE_DONE;
                }
            } else {
                t2_stable_cnt = 0;
            }
            break;

        /* ── 阶段3: 完成 ── */
        case T2_STAGE_DONE:
        default:
            break;
        }
        break;
    }
    case TASK_3: {
        switch (t3_stage) {

        /* ── 阶段0: 等待按键2启动 ── */
        case T3_STAGE_IDLE:
            track_stopped = 0;
            if (key.keynum == 2) {
                VisionControl_SetTarget(12.0f);
                key.start = 1;
                t3_stage  = T3_STAGE_TRACKING;
            }
            break;

        /* ── 阶段1: 正在循迹 + 视觉推杆12cm，等待停车 ── */
        case T3_STAGE_TRACKING:
            if (track_stopped) {
                t3_stage  = T3_STAGE_STOPPED;
            }
            break;

        /* ── 阶段2: 已停车，时间保留在OLED上 ── */
        case T3_STAGE_STOPPED:
        default:
            break;
        }
        break;
    }
    case TASK_4: {
        switch (t4_stage) {

        /* ── 阶段0: 等待按键2启动 ── */
        case T4_STAGE_IDLE:
            track_stopped = 0;
            if (key.keynum == 2) {
                VisionControl_ResetCalib();
                t4_stage  = T4_STAGE_CALIB;  /* 先等标定，不启动小车 */
            }
            break;

        /* ── 阶段1: 等待视觉自动标定完成 ── */
        case T4_STAGE_CALIB:
            if (VisionControl_IsCalibDone()) {
                key.start = 1;
                t4_stage  = T4_STAGE_TRACKING;
            }
            break;

        /* ── 阶段2: 正在循迹 + 视觉自动标定，等待停车 ── */
        case T4_STAGE_TRACKING:
            if (track_stopped) {
                t4_stage  = T4_STAGE_STOPPED;
            }
            break;

        /* ── 阶段2: 已停车，时间保留在OLED上 ── */
        case T4_STAGE_STOPPED:
        default:
            break;
        }
        break;
    }
    default:
        break;
    }
}

/**
 * @brief 检查视觉推杆是否应激活（当前任务非 IDLE 阶段）
 */
bool Task_IsVisionActive(void)
{
    if (key.task_id == 2) return (t2_stage != T2_STAGE_IDLE);
    if (key.task_id == 3) return (t3_stage != T3_STAGE_IDLE);
    if (key.task_id == 4) return (t4_stage != T4_STAGE_IDLE);
    return false;
}
