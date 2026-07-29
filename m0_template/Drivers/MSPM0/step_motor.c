#include "step_motor.h"
#include <math.h>

/*
 *  单定时器双通道步进电机驱动 (TIMG8, PA0=步进2, PA1=步进1)
 *
 *  方案: DDA (数字微分分析器) 软件脉冲生成
 *  TIMG8 以 100kHz 固定速率触发 ZERO 中断
 *  ISR 中用 31 位累加器合成两路独立频率的方波
 *
 *  一脉冲 = 0.05625° (1/32 微步)
 *  脉冲频率 = 角速度 / 0.05625
 */

/* ---- 常量 ---- */
#define DDA_BASE_FREQ     100000U            /* ISR 基频 100kHz */
#define DDA_SCALE_BIT     31                 /* 累加器精度 31 位 */
#define DDA_SCALE         (1UL << DDA_SCALE_BIT)

/* ---- 每电机状态 ---- */
typedef struct {
    uint32_t          step_rate;             /* 脉冲速率 = (f / Fbase) * 2^31 */
    uint32_t          accumulator;           /* 相位累加器 */
    volatile uint32_t step_remain;           /* 剩余步数 (位置模式用) */
    volatile uint8_t  running;               /* 启停标志 */
    volatile uint32_t isr_cnt;               /* ISR 计数 */
    GPIO_Regs        *port;                  /* GPIO 端口指针 */
    uint32_t          pin_mask;              /* 引脚掩码 */
} motor_ctx_t;

static motor_ctx_t motor[2];

/* ---- 内部辅助 ---- */

/* dir_set 内部实现: 不暴露给外部, 由 continuous_run 统一管理 */
static void dir_set(uint8_t direction, uint8_t stepper_id)
{
    if (stepper_id == 1) {
        if (direction == 0)
            DL_GPIO_clearPins(STEP_MOTOR_DIR1_PORT, STEP_MOTOR_DIR1_PIN);
        else
            DL_GPIO_setPins(STEP_MOTOR_DIR1_PORT, STEP_MOTOR_DIR1_PIN);
    } else {
        if (direction == 0)
            DL_GPIO_clearPins(STEP_MOTOR_DIR2_PORT, STEP_MOTOR_DIR2_PIN);
        else
            DL_GPIO_setPins(STEP_MOTOR_DIR2_PORT, STEP_MOTOR_DIR2_PIN);
    }
}


/* ================================================================
 *  step_motor_Init
 *
 *  SysConfig 已调用 PWM init (时钟 + 定时器模式)
 *  这里接管 PA0/PA1 → GPIO, 配置 100kHz ZERO 中断
 * ================================================================ */
void step_motor_Init(void)
{
    GPTIMER_Regs *tim = DCC_100_PWM1_INST;

    /* DIR 引脚初始拉高 */
    DL_GPIO_setPins(STEP_MOTOR_DIR2_PORT, STEP_MOTOR_DIR2_PIN);
    DL_GPIO_setPins(STEP_MOTOR_DIR1_PORT, STEP_MOTOR_DIR1_PIN);

    /* PA0 (CCP1) 和 PA1 (CCP0) 改回 GPIO 输出, 初始低 */
    DL_GPIO_initDigitalOutput(GPIO_DCC_100_PWM1_C1_IOMUX);  /* PA0 */
    DL_GPIO_initDigitalOutput(GPIO_DCC_100_PWM1_C0_IOMUX);  /* PA1 */
    DL_GPIO_clearPins(GPIO_DCC_100_PWM1_C1_PORT, GPIO_DCC_100_PWM1_C1_PIN);
    DL_GPIO_clearPins(GPIO_DCC_100_PWM1_C0_PORT, GPIO_DCC_100_PWM1_C0_PIN);

    motor[0].port     = GPIO_DCC_100_PWM1_C0_PORT;   /* 电机1 → PA1 */
    motor[0].pin_mask = GPIO_DCC_100_PWM1_C0_PIN;
    motor[1].port     = GPIO_DCC_100_PWM1_C1_PORT;   /* 电机2 → PA0 */
    motor[1].pin_mask = GPIO_DCC_100_PWM1_C1_PIN;

    /* 设置 PWM 周期 = 100kHz (10MHz / 100 = 100000Hz) */
    uint32_t load_val = DCC_100_PWM1_INST_CLK_FREQ / DDA_BASE_FREQ;
    DL_Timer_setLoadValue(tim, load_val);
    DL_Timer_setCaptureCompareValue(tim, load_val / 2, DL_TIMER_CC_0_INDEX);
    DL_Timer_setCaptureCompareValue(tim, load_val / 2, DL_TIMER_CC_1_INDEX);

    /* 使能 ZERO 中断 */
    DL_Timer_enableInterrupt(tim, DL_TIMER_INTERRUPT_ZERO_EVENT);
    DL_Timer_clearInterruptStatus(tim, DL_TIMER_INTERRUPT_ZERO_EVENT);

    NVIC_EnableIRQ(DCC_100_PWM1_INST_INT_IRQN);

    /* 启动定时器 */
    DL_Timer_startCounter(tim);
}


/* ================================================================
 *  step_set_speed
 *
 *  计算 DDA step_rate 并写入电机结构体
 *  speed: 角速度 (度/秒), 始终非负
 * ================================================================ */
void step_set_speed(float speed, uint8_t stepper_id)
{
    uint8_t idx = stepper_id - 1;
    uint32_t pulse_freq = (uint32_t)(speed / 0.05625f);
    if (pulse_freq == 0) pulse_freq = 1;

    /* step_rate = (pulse_freq / DDA_BASE_FREQ) * 2^31 */
    motor[idx].step_rate = (uint32_t)(((uint64_t)pulse_freq << DDA_SCALE_BIT)
                                      / DDA_BASE_FREQ);
}


/* ================================================================
 *  step_motor_start / step_motor_stop
 * ================================================================ */
void step_motor_start(uint8_t stepper_id)
{
    uint8_t idx = stepper_id - 1;
    motor[idx].running = 1;
}

void step_motor_stop(uint8_t stepper_id)
{
    uint8_t idx = stepper_id - 1;
    motor[idx].running = 0;
    motor[idx].accumulator = 0;
}


/* ================================================================
 *  step_motor_dir_set (外部可用)
 * ================================================================ */
void step_motor_dir_set(uint8_t direction, uint8_t stepper_id)
{
    dir_set(direction, stepper_id);
}


/* ================================================================
 *  step_motor_continuous_run
 *
 *  PID / 循迹 实时调速接口
 *  speed_dps > 0 → 正转, < 0 → 反转, | | < 0.06 → 停转
 * ================================================================ */
void step_motor_continuous_run(uint8_t stepper_id, float speed_dps)
{
    uint8_t idx = stepper_id - 1;
    float abs_speed = fabsf(speed_dps);

    /* 死区 */
    if (abs_speed < 0.06f) {
        if (motor[idx].running) {
            step_motor_stop(stepper_id);
        }
        return;
    }

    /* 方向 */
    uint8_t direction = (speed_dps >= 0.0f) ? 1 : 0;
    /* 右电机对装, DIR 取反 */
    if (stepper_id == 2) direction = !direction;
    dir_set(direction, stepper_id);

    step_set_speed(abs_speed, stepper_id);

    if (!motor[idx].running) {
        step_motor_start(stepper_id);
    }
}


/* ================================================================
 *  step_rotate_by (位置模式, 保留兼容)
 * ================================================================ */
void step_rotate_by(float degrees, uint8_t direction, uint8_t stepper_id)
{
    uint8_t idx = stepper_id - 1;

    dir_set(direction, stepper_id);

    uint32_t steps = (uint32_t)(degrees / 0.05625f);
    uint8_t was_stopped = (motor[idx].step_remain == 0);
    motor[idx].step_remain += steps;

    if (motor[idx].step_remain > 0) {
        if (was_stopped) {
            step_set_speed(30.0f, stepper_id);
        }
        motor[idx].running = 1;
    }
}


/* ================================================================
 *  TIMG8 统一 ISR
 *
 *  DDA 脉冲生成:
 *    accumulator += step_rate
 *    进位 → 翻转 GPIO (生成一个脉冲边沿)
 *
 *  每 2 次进位 = 1 个完整脉冲
 * ================================================================ */
void DCC_100_PWM1_INST_IRQHandler(void)
{
    uint32_t iidx;
    GPTIMER_Regs *tim = DCC_100_PWM1_INST;

    while ((iidx = DL_Timer_getPendingInterrupt(tim)) != 0) {
        /* ZERO 事件: 每个 PWM 周期触发一次 (100kHz) */
        if (iidx == DL_TIMER_IIDX_ZERO || iidx == DL_TIMER_IIDX_LOAD) {
            for (int ch = 0; ch < 2; ch++) {
                if (!motor[ch].running && motor[ch].step_remain == 0) {
                    continue;
                }

                uint32_t acc = motor[ch].accumulator + motor[ch].step_rate;
                motor[ch].accumulator = acc;

                /* 进位 → 翻转电平 */
                if (acc < motor[ch].step_rate) {   /* 加法溢出 = 进位 */
                    motor[ch].port->DOUTTGL31_0 = motor[ch].pin_mask;

                    motor[ch].isr_cnt++;

                    /* 位置模式步数递减 */
                    if (motor[ch].step_remain > 0) {
                        motor[ch].step_remain--;
                        if (motor[ch].step_remain == 0 && !motor[ch].running) {
                            /* 仅位置模式用完步数时停转 */
                        }
                    }
                }
            }
        }
    }
}
