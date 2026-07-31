#include "sys.h"
#include <stdlib.h>

/* ================================================================
 * 全局变量
 * ================================================================ */
static uint8_t  time_prev_start = 0;  /* key.start 上升沿检测 */
uint32_t track_start_ms  = 0;          /* 行驶起始时间 */
uint8_t  track_stopped   = 0;          /* 停车标志 */
float    track_final_sec = 0;          /* 停车时刻保留的时间 */
static uint8_t  decelerating    = 0;   /* 减速中标志 */
static uint32_t decel_start_ms  = 0;   /* 减速起始时间 */
static float    decel_start_spd = 0;   /* 减速起始速度 */
volatile uint32_t test_ms = 0;       /* 1ms 计数器 */
static char oled_buf[16];

/* ================================================================
 * OLED 显示
 * ================================================================ */
static void OLED_UpdateStatus(void)
{
    /* 循迹时间：Task1/3/4模式下按键启动后计时，停车后保留 */
    if (!time_prev_start && key.start && (key.task_id == 1 || key.task_id == 3 || key.task_id == 4)) {
        track_start_ms = test_ms;
        track_stopped   = 0;
    }
    time_prev_start = key.start;

    float sec;
    if (track_stopped) {
        sec = track_final_sec;
    } else if (key.start && (key.task_id == 1 || key.task_id == 3 || key.task_id == 4)) {
        sec = (test_ms - track_start_ms) / 1000.0f;
    } else {
        sec = 0.0f;
    }

    /* 第1行：Digtal位图 + 视觉距离 */
    sprintf(oled_buf, "%02X Vis:%.1f", Digtal, vision.distance_cm);
    OLED_ShowString(0, 0, (uint8_t *)oled_buf, 16);

    /* 第2行：行驶时间 */
    sprintf(oled_buf, "%.1fs", sec);
    OLED_ShowString(0, 2, (uint8_t *)oled_buf, 16);

    /* 第3行：当前任务 */
    sprintf(oled_buf, "Tsk:%d", key.task_id);
    OLED_ShowString(0, 4, (uint8_t *)oled_buf, 16);

    /* 第4行：当前任务名称 */
    sprintf(oled_buf, "%s", Task_GetName(key.task_id));
    OLED_ShowString(0, 6, (uint8_t *)oled_buf, 16);
}

/* ================================================================
 * 传感器读取
 * ================================================================ */
static uint8_t SensorUpdate(void)
{
    static uint32_t ir0_black_start_ms = 0;  /* ir[0] 消抖计时 */
    static uint32_t ir7_black_start_ms = 0;  /* ir[7] 消抖计时 */

    uint8_t ir[8] = {1, 1, 1, 1, 1, 1, 1, 1};  /* 默认全白，防止首帧未到 */
    IR_Read(ir);

    /* 最右边传感器单独消抖：连续 0.5s 见黑才有效 */
    if (ir[7] == 0) {
        if (ir7_black_start_ms == 0) ir7_black_start_ms = test_ms;
        ir[7] = ((test_ms - ir7_black_start_ms) >= 500) ? 0 : 1;
    } else {
        ir7_black_start_ms = 0;
        ir[7] = 1;
    }

    /* 最左边传感器同样消抖：连续 0.5s 见黑才有效 */
    if (ir[0] == 0) {
        if (ir0_black_start_ms == 0) ir0_black_start_ms = test_ms;
        ir[0] = ((test_ms - ir0_black_start_ms) >= 500) ? 0 : 1;
    } else {
        ir0_black_start_ms = 0;
        ir[0] = 1;
    }

    /* 最左边滤波：仅相邻 ir[1] 也为黑时 ir[0] 黑才有效 */
    if (ir[0] == 0 && ir[1] != 0) {
        ir[0] = 1;  /* 孤立黑 → 强制白 */
    }

    /* 最右边滤波：仅相邻 ir[6] 也为黑时 ir[7] 黑才有效 */
    if (ir[7] == 0 && ir[6] != 0) {
        ir[7] = 1;  /* 孤立黑 → 强制白 */
    }

    Digtal = (ir[0]<<7) | (ir[1]<<6) | (ir[2]<<5) | (ir[3]<<4)
           | (ir[4]<<3) | (ir[5]<<2) | (ir[6]<<1) | (ir[7]<<0);

    Get_err2();
    return Digtal;
}

/* ================================================================
 * 十字停车检测
 * 逻辑：300ms 滑动窗口内，累计 ≥3 路黑的时间 ≥50ms → 停车
 *      允许短暂掉线不重置，弯道不可能积累到 50ms
 * ================================================================ */
#define STOP_WINDOW_MS      300    /* 滑动窗口 ms */
#define STOP_BLACK_MIN      3      /* 最少黑线数 */
#define STOP_ACCUM_MIN_MS   50     /* 窗口内最少累计黑线时间 ms */
#define STOP_MIN_RUN_MS     18000  /* 最短运行时间 ms，防起步误判 */

static uint8_t CheckStop(uint8_t d)
{
    static uint32_t window_start  = 0;    /* 当前窗口起始时间 */
    static uint32_t black_acc_ms  = 0;    /* 窗口内累计见黑时间 */
    static uint32_t last_ms       = 0;    /* 上一帧时间 */

    if (!key.start || (key.task_id != 1 && key.task_id != 3 && key.task_id != 4)) {
        window_start = 0;
        black_acc_ms = 0;
        last_ms      = 0;
        return 0;
    }

    uint32_t now = test_ms;
    uint32_t dt  = (last_ms > 0) ? (now - last_ms) : 0;
    last_ms = now;

    if (window_start == 0) window_start = now;

    /* 统计当前帧有几路见黑（bit=0 为黑线） */
    uint8_t black_cnt = 0;
    for (uint8_t i = 0; i < 8; i++) {
        if (!(d & (1 << i))) black_cnt++;
    }

    /* 本帧见黑 → 累加时间 */
    if (black_cnt >= STOP_BLACK_MIN) {
        black_acc_ms += dt;
    }

    /* 窗口到期 → 判断 */
    if (now - window_start >= STOP_WINDOW_MS) {
        if (black_acc_ms >= STOP_ACCUM_MIN_MS
            && (now - track_start_ms) >= STOP_MIN_RUN_MS) {
            track_final_sec = (now - track_start_ms) / 1000.0f;
            track_stopped = 1;
            decelerating   = 1;
            decel_start_ms = now;
            decel_start_spd = (float)base_speed;
            key.start = 0;
            return 1;
        }
        /* 不满足 → 重置窗口 */
        window_start = now;
        black_acc_ms = 0;
    }

    return 0;
}

/* ================================================================
 * main
 * ================================================================ */

int main(void)
{
    SYSCFG_DL_init();
    SysTick_Init();
    OLED_Init();

    /* 步进电机 + 循迹 + 视觉 */
    step_motor_Init();
    StepTrack_Init();
    Vision_Init();
    VisionControl_Init();
    Task_Init();

    /* 按键默认值 */
    key.task_id = 0;

    /* LED */
    LED4_High;
    LED3_High;

    Interrupt_Init();

    /* 1ms 定时器 */
    NVIC_EnableIRQ(TIMER_xunji_pid_INST_INT_IRQN);

    /* 红外循迹传感器 */
    IR_Init();

    while (1)
    {
        key_work();                     /* 1. 按键扫描 */

        uint8_t d = SensorUpdate();     /* 2. IR 传感器 + 误差 */

        /* 立即停车 */
        if (decelerating) {
            step_motor_stop(1);
            step_motor_stop(2);
            step_motor_stop(3);
            decelerating = 0;
            continue;
        }

        OLED_UpdateStatus();            /* 4. OLED: 时间 / Digtal / Spd / Lap */

        if (CheckStop(d)) continue;     /* 5. 十字停车 */

        /* 6. 步进电机循迹（仅 Task 1/3/4 需要） */
        if (key.task_id == 1 || key.task_id == 3 || key.task_id == 4) {
            StepTrack_Run();
        }

        /* 7. 视觉推杆控制（仅激活中的 Task 2/3/4） */
        if (Task_IsVisionActive()) {
            VisionControl_Run();
        }

        Task_Run();                     /* 8. 任务调度 */
    } 
}

/* ================================================================
 * 1ms 定时器 ISR
 * ================================================================ */
void TIMER_xunji_pid_INST_IRQHandler(void)
{
    delay_flag++;
    test_ms++;
}
