#include "sys.h"
#include <stdlib.h>

/* ================================================================
 * 全局变量
 * ================================================================ */
static uint8_t  time_prev_start = 0;  /* key.start 上升沿检测 */
volatile uint32_t track_start_ms  = 0;  /* 行驶起始时间 */
uint8_t  track_stopped   = 0;          /* 停车标志 */
float    track_final_sec = 0;          /* 停车时刻保留的时间 */
uint8_t  decelerating    = 0;          /* 减速中标志 */
volatile uint32_t test_ms = 0;         /* 1ms 计数器 */
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
    uint8_t ir[8] = {1, 1, 1, 1, 1, 1, 1, 1};  /* 默认全白，防首帧未到 */

    IR_Read(ir);

    /* 左1传感器已坏，强制白避免干扰循迹和停车检测 */
    ir[0] = 1;

    /* 最外侧传感器邻域门控: 黑线必有宽度，单路孤立即为噪声 */
    if (ir[7] == 0 && ir[6] != 0) ir[7] = 1;   /* 右1孤黑 → 滤除 */

    Digtal = (ir[0]<<7) | (ir[1]<<6) | (ir[2]<<5) | (ir[3]<<4)
           | (ir[4]<<3) | (ir[5]<<2) | (ir[6]<<1) | (ir[7]<<0);

    Get_err2();
    return Digtal;
}

/* ================================================================
 * 十字停车检测
 * 100ms内最右边5路（ir[3]~ir[7]）累积 ≥5路亮起 → 触发停车
 * ================================================================ */
#define STOP_MIN_RUN_T1_MS  15000  /* 任务1 最短运行时间 ms */
#define STOP_MIN_RUN_T3_MS  20000  /* 任务3 最短运行时间 ms（20s内不可停车） */
#define STOP_ACCUM_WINDOW_MS  120  /* 累积窗口 ms */

static void CheckStop(uint8_t d)
{
    static uint32_t acc_start_ms = 0;    /* 累积起始时刻 */
    static uint8_t  acc_bits     = 0;    /* 累积的黑位 */

    if (!key.start || (key.task_id != 1 && key.task_id != 3 && key.task_id != 4)) {
        acc_start_ms = 0;
        acc_bits     = 0;
        return;
    }

    if (decelerating) return;           /* 已在减速，不重复触发 */

    uint32_t now = test_ms;

    /* 最短运行时间保护（按任务区分） */
    {
        uint32_t min_run = (key.task_id == 3) ? STOP_MIN_RUN_T3_MS : STOP_MIN_RUN_T1_MS;
        if ((now - track_start_ms) < min_run) {
            acc_start_ms = 0;
            acc_bits     = 0;
            return;
        }
    }

    /* 最右边5路黑位: ir[3]~ir[7] 对应 ~d 的 bit4~bit0, 1=黑 */
    uint8_t right_bits = ((uint8_t)~d) & 0x1F;

    /* 100ms 超时重置 */
    if (acc_start_ms != 0 && (now - acc_start_ms) > STOP_ACCUM_WINDOW_MS) {
        acc_start_ms = 0;
        acc_bits     = 0;
    }

    if (right_bits != 0) {
        if (acc_start_ms == 0) acc_start_ms = now;
        acc_bits |= right_bits;

        /* 统计累积黑路数 */
        uint8_t n = acc_bits;
        n = (n & 0x55) + ((n >> 1) & 0x55);
        n = (n & 0x33) + ((n >> 2) & 0x33);
        n = (n & 0x0F) + (n >> 4);

        if (n >= 5) {
            track_final_sec = (now - track_start_ms) / 1000.0f;
            track_stopped = 1;
            decelerating   = 1;         /* 触发减速，key.start 保持 1 */
            acc_start_ms = 0;
            acc_bits     = 0;
        }
    }
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

        CheckStop(d);                   /* 3. 十字停车检测 */

        OLED_UpdateStatus();            /* 4. OLED: 时间 / Digtal / Spd / Lap */

        /* 5. 步进电机循迹（仅 Task 1/3/4 需要，减速由 StepTrack_Run 内部处理） */
        if (key.task_id == 1 || key.task_id == 3 || key.task_id == 4) {
            StepTrack_Run();
        }

        /* 6. 视觉推杆控制（仅激活中的 Task 2/3/4） */
        if (Task_IsVisionActive()) {
            VisionControl_Run();
        }

        Task_Run();                     /* 7. 任务调度 */
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
