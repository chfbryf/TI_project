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
    uint8_t ir[8];

    IR_Read(ir);

    Digtal = (ir[0]<<7) | (ir[1]<<6) | (ir[2]<<5) | (ir[3]<<4)
           | (ir[4]<<3) | (ir[5]<<2) | (ir[6]<<1) | (ir[7]<<0);

    Get_err2();
    return Digtal;
}

/* ================================================================
 * 十字停车检测
 * 停车标志：≥6路连续见黑 100ms → 停车
 * ================================================================ */
#define STOP_MIN_RUN_MS     18000  /* 最短运行时间 ms，防起步误判 */
#define STOP_HOLD_MS        100   /* 连续满足条件 ms */

static uint8_t CheckStop(uint8_t d)
{
    static uint32_t stop_start_ms = 0;  /* 满足条件起始时间 */

    if (!key.start || (key.task_id != 1 && key.task_id != 3 && key.task_id != 4)) {
        stop_start_ms = 0;
        return 0;
    }

    uint32_t now = test_ms;

    /* 最短运行时间保护，防止起步误判 */
    if ((now - track_start_ms) < STOP_MIN_RUN_MS) {
        return 0;
    }

    /* 全8路计数 ≥6 → 停车 */
    uint8_t n = d;
    n = (n & 0x55) + ((n >> 1) & 0x55);
    n = (n & 0x33) + ((n >> 2) & 0x33);
    n = (n & 0x0F) + (n >> 4);
    if (n >= 6) {
        if (stop_start_ms == 0) stop_start_ms = now;
        if ((now - stop_start_ms) >= STOP_HOLD_MS) {
            track_final_sec = (now - track_start_ms) / 1000.0f;
            track_stopped = 1;
            decelerating   = 1;
            decel_start_ms = now;
            decel_start_spd = (float)base_speed;
            key.start = 0;
            return 1;
        }
    } else {
        stop_start_ms = 0;  /* 不满足 → 重置计时 */
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
        /* 非阻塞 50ms 节流：与传感器 UART 帧率匹配 */
        static uint32_t last_loop_ms = 0;
        if (test_ms - last_loop_ms < 50) continue;
        last_loop_ms = test_ms;

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
