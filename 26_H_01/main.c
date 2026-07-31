#include "sys.h"
#include <stdlib.h>

/* ================================================================
 * 全局变量
 * ================================================================ */
static uint8_t  time_prev_start = 0;  /* key.start 上升沿检测 */
static uint32_t track_start_ms  = 0;  /* 行驶起始时间 */
static uint8_t  track_stopped   = 0;  /* 停车标志 */
static float    track_final_sec = 0;  /* 停车时刻保留的时间 */
static uint8_t  decelerating    = 0;  /* 减速中标志 */
static uint32_t decel_start_ms  = 0;  /* 减速起始时间 */
static float    decel_start_spd = 0;  /* 减速起始速度 */
volatile uint32_t test_ms = 0;       /* 1ms 计数器 */
static char oled_buf[16];

/* ================================================================
 * OLED 显示
 * ================================================================ */
static void OLED_UpdateStatus(void)
{
    /* 循迹时间：按键启动后计时，停车后保留 */
    if (!time_prev_start && key.start) {
        track_start_ms = test_ms;
        track_stopped   = 0;
    }
    time_prev_start = key.start;

    float sec;
    if (track_stopped) {
        sec = track_final_sec;
    } else if (key.start) {
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

    /* 第4行：圈数 */
    sprintf(oled_buf, "Lap:%d", key.quan);
    OLED_ShowString(0, 6, (uint8_t *)oled_buf, 16);
}

/* ================================================================
 * 传感器读取
 * ================================================================ */
static uint8_t SensorUpdate(void)
{
    static uint8_t ir7_db_cnt = 0;

    uint8_t ir[8] = {1, 1, 1, 1, 1, 1, 1, 1};  /* 默认全白，防止首帧未到 */
    IR_Read(ir);

    /* 最右边传感器单独消抖：连续3帧见黑才有效 */
    if (ir[7] == 0) {
        if (ir7_db_cnt < 3) ir7_db_cnt++;
    } else {
        ir7_db_cnt = 0;
    }
    ir[7] = (ir7_db_cnt >= 3) ? 0 : 1;

    Digtal = (ir[0]<<7) | (ir[1]<<6) | (ir[2]<<5) | (ir[3]<<4)
           | (ir[4]<<3) | (ir[5]<<2) | (ir[6]<<1) | (ir[7]<<0);

    Get_err2();
    return Digtal;
}

/* ================================================================
 * 十字停车检测
 * ================================================================ */
static uint8_t CheckStop(uint8_t d)
{
    static uint8_t  black_seen    = 0;
    static uint32_t window_start  = 0;

    if (!key.start) {                      /* 未启动时清空窗口，避免误触发 */
        black_seen   = 0;
        window_start = 0;
        return 0;
    }

    black_seen |= ~d;                      /* 累积本窗口内见过的黑线位置 */

    if (test_ms - window_start >= 300) {   /* 100ms 窗口到 */
        uint8_t cnt = 0;
        for (uint8_t i = 0; i < 8; i++) {
            if (black_seen & (1 << i)) cnt++;
        }
        if (cnt >= 4 && (test_ms - track_start_ms) >= 18000) {
            track_final_sec = (test_ms - track_start_ms) / 1000.0f;
            track_stopped = 1;
            decelerating   = 1;
            decel_start_ms = test_ms;
            decel_start_spd = (float)base_speed;
            key.start = 0;
            return 1;
        }
        /* 重置窗口 */
        black_seen   = 0;
        window_start = test_ms;
    }

    if (window_start == 0) window_start = test_ms;
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
    key.task_id = 1;

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
            decelerating = 0;
            continue;
        }

        OLED_UpdateStatus();            /* 4. OLED: 时间 / Digtal / Spd / Lap */

        if (CheckStop(d)) continue;     /* 5. 十字停车 */

        StepTrack_Run();                /* 6. 步进电机循迹 */

        VisionControl_Run();            /* 7. 视觉推杆控制 */

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
