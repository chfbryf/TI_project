#include "step_motor.h"
#include <math.h>

/*
 *  三路 PWM 步进电机驱动
 *  - TIMG8 → PA1 = 电机1, TIMG6 → PB2 = 电机3
 *  - TIMA0 → PA0 = 电机2 (右轮对装方向取反)
 *
 *  方案: 硬件 PWM 直接输出步进脉冲
 *  - 定时器周期 = CLK_FREQ / 脉冲频率  (速度越快, 周期越小)
 *  - 占空比 50%
 *  - 启动/停止 = 启停定时器
 *
 *  一脉冲 = 0.05625° (1/32 微步)
 *  脉冲频率 = 角速度 / 0.05625
 */

volatile uint32_t step_remain_1 = 0;
volatile uint32_t step_remain_2 = 0;
volatile uint32_t step_remain_3 = 0;
volatile uint32_t isr_cnt_1 = 0;
volatile uint32_t isr_cnt_2 = 0;
volatile uint32_t isr_cnt_3 = 0;

/* 电机启停状态追踪，避免每轮 PID 重复启停定时器 */
static uint8_t motor_running[3] = {0, 0, 0};


/* ================================================================
 *  step_motor_Init
 * ================================================================ */
void step_motor_Init(void)
{
    /* DIR 引脚初始拉高 */
    DL_GPIO_setPins(STEP_MOTOR_DIR1_PORT, STEP_MOTOR_DIR1_PIN);
    DL_GPIO_setPins(STEP_MOTOR_DIR2_PORT, STEP_MOTOR_DIR2_PIN);
    DL_GPIO_setPins(STEP_MOTOR_DIR3_PORT, STEP_MOTOR_DIR3_PIN);

    /* EN 使能 (高电平有效) */
    DL_GPIO_setPins(STEP_MOTOR_EN1_PORT, STEP_MOTOR_EN1_PIN);
    DL_GPIO_setPins(STEP_MOTOR_EN2_PORT, STEP_MOTOR_EN2_PIN);
    DL_GPIO_setPins(STEP_MOTOR_EN3_PORT, STEP_MOTOR_EN3_PIN);

    NVIC_EnableIRQ(DCC_100_PWM1_INST_INT_IRQN);
    NVIC_EnableIRQ(DCC_100_PWM2_INST_INT_IRQN);
    NVIC_EnableIRQ(DCC_100_PWM3_INST_INT_IRQN);
}


/* ================================================================
 *  step_motor_dir_set
 * ================================================================ */
void step_motor_dir_set(uint8_t direction, uint8_t stepper_id)
{
    if (stepper_id == 1) {
        if (direction == 0)
            DL_GPIO_clearPins(STEP_MOTOR_DIR1_PORT, STEP_MOTOR_DIR1_PIN);
        else
            DL_GPIO_setPins(STEP_MOTOR_DIR1_PORT, STEP_MOTOR_DIR1_PIN);
    } else if (stepper_id == 2) {
        if (direction == 0)
            DL_GPIO_clearPins(STEP_MOTOR_DIR2_PORT, STEP_MOTOR_DIR2_PIN);
        else
            DL_GPIO_setPins(STEP_MOTOR_DIR2_PORT, STEP_MOTOR_DIR2_PIN);
    } else {
        if (direction == 0)
            DL_GPIO_clearPins(STEP_MOTOR_DIR3_PORT, STEP_MOTOR_DIR3_PIN);
        else
            DL_GPIO_setPins(STEP_MOTOR_DIR3_PORT, STEP_MOTOR_DIR3_PIN);
    }
}


/* ================================================================
 *  step_motor_start / step_motor_stop
 * ================================================================ */
void step_motor_start(uint8_t stepper_id)
{
    if (stepper_id == 1) {
        NVIC_EnableIRQ(DCC_100_PWM1_INST_INT_IRQN);
        DL_Timer_startCounter(DCC_100_PWM1_INST);
    }
    if (stepper_id == 2) {
        NVIC_EnableIRQ(DCC_100_PWM2_INST_INT_IRQN);
        DL_Timer_startCounter(DCC_100_PWM2_INST);
    }
    if (stepper_id == 3) {
        NVIC_EnableIRQ(DCC_100_PWM3_INST_INT_IRQN);
        DL_Timer_startCounter(DCC_100_PWM3_INST);
    }
}

void step_motor_stop(uint8_t stepper_id)
{
    if (stepper_id == 1) {
        DL_Timer_stopCounter(DCC_100_PWM1_INST);
    }
    if (stepper_id == 2) {
        DL_Timer_stopCounter(DCC_100_PWM2_INST);
    }
    if (stepper_id == 3) {
        DL_Timer_stopCounter(DCC_100_PWM3_INST);
    }
}


/* ================================================================
 *  step_set_speed
 *
 *  speed: 角速度 (度/秒), 始终非负
 *  定时器周期 = CLK_FREQ / 脉冲频率, 限幅 [800, 65535]
 *    800  → 最大步进频率 12.5kHz → 703 deg/s
 *    65535 → 最小步进频率  153Hz  → 8.6 deg/s
 * ================================================================ */
void step_set_speed(float speed, uint8_t stepper_id)
{
    if (stepper_id == 1) {
        uint32_t frequency = (uint32_t)(speed / 0.05625f);
        frequency = frequency > 0 ? frequency : 1;
        uint32_t period = DCC_100_PWM1_INST_CLK_FREQ / frequency;
        period = period < 65535 ? period : 65535;
        period = period > 800 ? period : 800;

        DL_Timer_setLoadValue(DCC_100_PWM1_INST, period);
        DL_Timer_setCaptureCompareValue(DCC_100_PWM1_INST, period / 2,
                                        GPIO_DCC_100_PWM1_C0_IDX);
    }

    if (stepper_id == 2) {
        uint32_t frequency = (uint32_t)(speed / 0.05625f);
        frequency = frequency > 0 ? frequency : 1;
        uint32_t period = DCC_100_PWM2_INST_CLK_FREQ / frequency;
        period = period < 65535 ? period : 65535;
        period = period > 800 ? period : 800;

        DL_Timer_setLoadValue(DCC_100_PWM2_INST, period);
        DL_Timer_setCaptureCompareValue(DCC_100_PWM2_INST, period / 2,
                                        GPIO_DCC_100_PWM2_C0_IDX);
    }

    if (stepper_id == 3) {
        uint32_t frequency = (uint32_t)(speed / 0.05625f);
        frequency = frequency > 0 ? frequency : 1;
        uint32_t period = DCC_100_PWM3_INST_CLK_FREQ / frequency;
        period = period < 65535 ? period : 65535;
        period = period > 800 ? period : 800;

        DL_Timer_setLoadValue(DCC_100_PWM3_INST, period);
        DL_Timer_setCaptureCompareValue(DCC_100_PWM3_INST, period / 2,
                                        GPIO_DCC_100_PWM3_C0_IDX);
    }
}


/* ================================================================
 *  step_rotate_by (位置模式, 保留兼容)
 * ================================================================ */
void step_rotate_by(float degrees, uint8_t direction, uint8_t stepper_id)
{
    step_motor_dir_set(direction, stepper_id);

    if (stepper_id == 1) {
        uint32_t steps = (uint32_t)(degrees / 0.05625f);
        uint8_t was_stopped = (step_remain_1 == 0);
        step_remain_1 += steps;
        if (step_remain_1 > 0) {
            if (was_stopped) {
                step_set_speed(30.0f, 1);
            }
            step_motor_start(1);
        }
    } else if (stepper_id == 2) {
        uint32_t steps = (uint32_t)(degrees / 0.05625f);
        uint8_t was_stopped = (step_remain_2 == 0);
        step_remain_2 += steps;
        if (step_remain_2 > 0) {
            if (was_stopped) {
                step_set_speed(30.0f, 2);
            }
            step_motor_start(2);
        }
    } else if (stepper_id == 3) {
        uint32_t steps = (uint32_t)(degrees / 0.05625f);
        uint8_t was_stopped = (step_remain_3 == 0);
        step_remain_3 += steps;
        if (step_remain_3 > 0) {
            if (was_stopped) {
                step_set_speed(30.0f, 3);
            }
            step_motor_start(3);
        }
    }
}


/* ================================================================
 *  step_motor_continuous_run
 *
 *  PID / 循迹 实时调速接口
 *  speed_dps > 0 → 正转, < 0 → 反转, | | < 0.06 → 停转
 *
 *  只在启停边缘做 start/stop，避免每次 PID 都调用 startCounter。
 * ================================================================ */
void step_motor_continuous_run(uint8_t stepper_id, float speed_dps)
{
    uint8_t idx = stepper_id - 1;
    float abs_speed = fabsf(speed_dps);

    /* 死区 */
    if (abs_speed < 0.06f) {
        if (motor_running[idx]) {
            step_motor_stop(stepper_id);
            motor_running[idx] = 0;
        }
        return;
    }

    /* 方向 */
    uint8_t direction = (speed_dps >= 0.0f) ? 1 : 0;
    /* 右电机对装, DIR 取反 */
    if (stepper_id == 2 || stepper_id == 3) direction = !direction;

    /* 先更新方向，再更新速度 */
    step_motor_dir_set(direction, stepper_id);
    step_set_speed(abs_speed, stepper_id);

    /* 仅在首次启动时调用 step_motor_start */
    if (!motor_running[idx]) {
        step_motor_start(stepper_id);
        motor_running[idx] = 1;
    }
}


/* ================================================================
 *  PWM 中断
 *
 *  ISR 中只做步数计数（供位置模式 step_rotate_by 使用）。
 *  连续调速模式不需要 ISR 做任何事，硬件 PWM 自动输出脉冲。
 * ================================================================ */

void DCC_100_PWM2_INST_IRQHandler(void)
{
    uint32_t iidx;
    while ((iidx = DL_Timer_getPendingInterrupt(DCC_100_PWM2_INST)) != 0) {
        isr_cnt_2++;
        if (step_remain_2 > 0) {
            step_remain_2--;
            if (step_remain_2 == 0) {
                step_motor_stop(2);
            }
        }
    }
}

void DCC_100_PWM1_INST_IRQHandler(void)
{
    uint32_t iidx;
    while ((iidx = DL_Timer_getPendingInterrupt(DCC_100_PWM1_INST)) != 0) {
        isr_cnt_1++;
        if (step_remain_1 > 0) {
            step_remain_1--;
            if (step_remain_1 == 0) {
                step_motor_stop(1);
            }
        }
    }
}

void DCC_100_PWM3_INST_IRQHandler(void)
{
    uint32_t iidx;
    while ((iidx = DL_Timer_getPendingInterrupt(DCC_100_PWM3_INST)) != 0) {
        isr_cnt_3++;
        if (step_remain_3 > 0) {
            step_remain_3--;
            if (step_remain_3 == 0) {
                step_motor_stop(3);
            }
        }
    }
}
