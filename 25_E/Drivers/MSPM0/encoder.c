#include "encoder.h"
#include "clock.h"

#include <stdbool.h>
#include <stdint.h>
#include <math.h>


/*==============================================================================
 *                             用户可调整参数
 *============================================================================*/

/*
 * 编码器每转有效计数数量。
 *
 * 注意：这里必须填写“实际进入 Encodering() 的有效脉冲数”，
 * 不一定等于编码器铭牌上标注的线数。
 *
 * 例如编码器为 22 线：
 *
 * 1. A 相只检测上升沿：
 *      ENCODER_PPR = 22
 *
 * 2. A 相同时检测上升沿和下降沿：
 *      ENCODER_PPR = 44
 *
 * 3. A、B 两相都检测双边沿，即四倍频：
 *      ENCODER_PPR = 88
 *
 * 当前程序只处理 A 相中断，因此一般设置为 22 或 44，
 * 具体取决于 GPIO 中断边沿配置。
 */
#define ENCODER_PPR                22.0f

/* 电机减速箱减速比 */
#define GEAR_RATIO                 30.0f

/*
 * 一阶低通滤波系数，范围为 0～1。
 *
 * 数值越小：
 *   速度越平滑，但响应越慢。
 *
 * 数值越大：
 *   响应越快，但速度波动也会更明显。
 *
 * 推荐范围：0.15～0.35
 */
#define FILTER_ALPHA               0.25f

/*
 * 输出轴允许的最大角速度，单位 rad/s。
 *
 * 超过这个范围的测量结果会被限制，
 * 防止异常脉冲产生不合理的巨大速度。
 */
#define MAX_SPEED_RAD_S            60.0f

/*
 * 滤波速度每调用一次允许变化的最大值，单位 rad/s。
 *
 * 这个参数可以限制单次异常值对最终速度的影响。
 *
 * 数值越小：
 *   抗尖峰能力越强，但加减速响应越慢。
 *
 * 数值越大：
 *   响应越快，但抑制尖峰的能力越弱。
 */
#define MAX_DELTA_PER_CALL         8.0f

/*
 * 编码器相邻有效脉冲允许的最小时间，单位微秒。
 *
 * 小于这个时间的脉冲会被认为是干扰。
 *
 * 原代码使用 100us。
 * 如果 A 相使用双边沿中断，在高速时正常脉冲间隔可能小于 100us，
 * 导致正常脉冲被错误删除，所以这里暂时修改为 30us。
 *
 * 不建议在没有确认中断边沿配置的情况下继续增大。
 */
#define MIN_PULSE_US               30U

/*
 * 连续检测到多少个反方向脉冲，才确认电机真的换向。
 *
 * 2：换向响应较快，但抗干扰稍弱。
 * 3：推荐值，兼顾响应速度和抗干扰能力。
 * 4：抗干扰更强，但真正换向时会多丢弃几个脉冲。
 */
#define DIRECTION_CONFIRM_PULSES   3U

/*
 * 当计算速度绝对值小于该值时，直接归零。
 *
 * 用于避免电机已经停止时，显示一个非常小的残余速度。
 */
#define SPEED_ZERO_EPSILON         0.001f

/* 2π，用于将每秒转数换算为 rad/s */
#define TWO_PI                     6.28318530718f


/*==============================================================================
 *                              编码器状态变量
 *============================================================================*/

/*
 * 编码器累计计数。
 *
 * 右编码器：
 *   正转时增加，反转时减少。
 *
 * 左编码器：
 *   由于电机镜像安装，方向判断逻辑与右编码器相反，
 *   但最终仍保证正转时增加，反转时减少。
 */
static volatile int64_t encoder_R = 0;
static volatile int64_t encoder_L = 0;

/*
 * 当前已经确认的稳定方向：
 *
 *  1：正转
 * -1：反转
 *  0：上电后还没有检测到有效方向
 *
 * 不再使用原代码中的 2 和 -2。
 */
static volatile int8_t direction_R = 0;
static volatile int8_t direction_L = 0;

/*
 * 正在等待确认的候选方向。
 *
 * 当本次脉冲方向与稳定方向不一致时，
 * 不会马上改变 direction，而是先记录到 candidate_direction。
 */
static volatile int8_t candidate_direction_R = 0;
static volatile int8_t candidate_direction_L = 0;

/* 候选方向已经连续出现的次数 */
static volatile uint8_t candidate_count_R = 0;
static volatile uint8_t candidate_count_L = 0;

/*
 * 编码器脉冲时间戳，单位微秒。
 *
 * t0：最近一次被接受的有效脉冲时间。
 * t1：上一次被接受的有效脉冲时间。
 *
 * 未通过方向确认的异常脉冲不会修改这两个时间戳，
 * 防止异常脉冲污染速度周期。
 */
static volatile uint64_t t0_R = 0;
static volatile uint64_t t1_R = 0;

static volatile uint64_t t0_L = 0;
static volatile uint64_t t1_L = 0;

/* 左右电机滤波后的速度，单位 rad/s */
static float speed_R_filt = 0.0f;
static float speed_L_filt = 0.0f;


/*==============================================================================
 *                              内部辅助函数
 *============================================================================*/

/**
 * @brief 将浮点数限制在给定范围内
 *
 * @param value 当前数值
 * @param minimum 最小允许值
 * @param maximum 最大允许值
 *
 * @return 限制后的数值
 */
static float Encoder_ClampFloat(
    float value,
    float minimum,
    float maximum)
{
    if(value > maximum)
    {
        return maximum;
    }

    if(value < minimum)
    {
        return minimum;
    }

    return value;
}


/**
 * @brief 对编码器方向进行连续脉冲确认
 *
 * 当前程序不允许一个脉冲直接改变电机方向。
 *
 * 当 observed_direction 与 stable_direction 不同时：
 *
 * 1. 第一次出现：记录候选方向，返回 false；
 * 2. 第二次出现：候选计数增加，返回 false；
 * 3. 连续达到 DIRECTION_CONFIRM_PULSES 次：
 *    正式改变稳定方向，返回 true。
 *
 * 这样可以防止单个 A/B 相干扰脉冲导致速度瞬间反号。
 *
 * @param stable_direction 当前已经确认的方向
 * @param candidate_direction 正在等待确认的候选方向
 * @param candidate_count 候选方向连续出现次数
 * @param observed_direction 本次脉冲观察到的方向，只能为 1 或 -1
 *
 * @return true
 *         本次脉冲方向有效，可以参与计数和测速。
 *
 * @return false
 *         本次方向尚未确认，暂时丢弃该脉冲。
 */
static bool Encoder_ConfirmDirection(
    volatile int8_t *stable_direction,
    volatile int8_t *candidate_direction,
    volatile uint8_t *candidate_count,
    int8_t observed_direction)
{
    /*
     * 上电后的第一个有效脉冲。
     *
     * 此时还没有稳定方向，可以直接使用本次观察方向
     * 建立初始方向。
     */
    if(*stable_direction == 0)
    {
        *stable_direction = observed_direction;
        *candidate_direction = 0;
        *candidate_count = 0;

        return true;
    }

    /*
     * 本次方向与稳定方向相同。
     *
     * 说明电机仍然按照原方向运行，
     * 清除之前可能存在的候选换向状态。
     */
    if(observed_direction == *stable_direction)
    {
        *candidate_direction = 0;
        *candidate_count = 0;

        return true;
    }

    /*
     * 本次方向与稳定方向不同。
     *
     * 如果它也与之前的候选方向不同，
     * 说明这是一个新的候选换向过程。
     */
    if(observed_direction != *candidate_direction)
    {
        *candidate_direction = observed_direction;
        *candidate_count = 1;

        return false;
    }

    /*
     * 本次方向与候选方向相同，
     * 表示连续多次检测到了相同的新方向。
     */
    if(*candidate_count < 255U)
    {
        (*candidate_count)++;
    }

    /*
     * 尚未达到换向确认次数。
     *
     * 暂时认为可能是干扰脉冲，
     * 不参与编码器计数和速度周期计算。
     */
    if(*candidate_count < DIRECTION_CONFIRM_PULSES)
    {
        return false;
    }

    /*
     * 连续出现足够多次相同的新方向，
     * 确认电机确实发生了换向。
     */
    *stable_direction = observed_direction;
    *candidate_direction = 0;
    *candidate_count = 0;

    return true;
}


/**
 * @brief 计算并滤波编码器速度
 *
 * 速度计算使用相邻有效脉冲的周期。
 *
 * 当脉冲停止后：
 *
 *   T_elapsed 会逐渐增大
 *       ↓
 *   计算速度逐渐减小
 *       ↓
 *   最终速度平滑回到零
 *
 * 所有方向，包括真正换向后的反方向速度，
 * 都必须经过同一套限幅、低通滤波和变化率限制。
 *
 * 不允许像原代码一样，在换向时直接返回负的滤波速度。
 *
 * @param direction 当前稳定方向
 * @param pulse_time_now 最近一次有效脉冲时间
 * @param pulse_time_last 上一次有效脉冲时间
 * @param current_time 当前系统时间
 * @param filtered_speed 滤波速度状态
 *
 * @return 滤波后的速度，单位 rad/s
 */
static float Encoder_CalculateSpeed(
    int8_t direction,
    uint64_t pulse_time_now,
    uint64_t pulse_time_last,
    uint64_t current_time,
    float *filtered_speed)
{
    float raw_speed = 0.0f;

    /*
     * 至少需要两个有效脉冲，才能计算完整的脉冲周期。
     *
     * 上电后还没有脉冲，或者只收到一个脉冲时，
     * 暂时将原始速度目标设为零。
     */
    if((direction != 0) &&
       (pulse_time_now != 0U) &&
       (pulse_time_last != 0U) &&
       (pulse_time_now >= pulse_time_last) &&
       (current_time >= pulse_time_now))
    {
        uint64_t pulse_period_us;
        uint64_t elapsed_time_us;
        uint64_t speed_period_us;

        /*
         * 最近两个有效脉冲之间的时间。
         */
        pulse_period_us = pulse_time_now - pulse_time_last;

        /*
         * 距离最近一次有效脉冲已经过去的时间。
         */
        elapsed_time_us = current_time - pulse_time_now;

        /*
         * 正常连续转动时，使用最近的脉冲周期。
         *
         * 如果脉冲停止，elapsed_time_us 会逐渐变大，
         * 因此速度会逐渐下降。
         */
        if(elapsed_time_us > pulse_period_us)
        {
            speed_period_us = elapsed_time_us;
        }
        else
        {
            speed_period_us = pulse_period_us;
        }

        /* 防止除零 */
        if(speed_period_us == 0U)
        {
            speed_period_us = 1U;
        }

        /*
         * 计算角速度：
         *
         * 每秒脉冲数：
         *     1000000 / speed_period_us
         *
         * 电机轴每秒转数：
         *     每秒脉冲数 / ENCODER_PPR
         *
         * 输出轴每秒转数：
         *     电机轴每秒转数 / GEAR_RATIO
         *
         * 输出轴角速度：
         *     输出轴每秒转数 × 2π
         */
        raw_speed =
            1000000.0f /
            (float)speed_period_us /
            ENCODER_PPR /
            GEAR_RATIO *
            TWO_PI;

        if(direction < 0)
        {
            raw_speed = -raw_speed;
        }
    }

    /*
     * 原始速度硬限幅。
     *
     * 即使时间戳或者脉冲出现异常，
     * 也不能产生超过机械最大速度的输出。
     */
    raw_speed = Encoder_ClampFloat(
        raw_speed,
        -MAX_SPEED_RAD_S,
        MAX_SPEED_RAD_S);

    /*
     * 一阶低通滤波。
     *
     * 先计算本次希望改变的速度量。
     */
    float delta =
        FILTER_ALPHA *
        (raw_speed - *filtered_speed);

    /*
     * 限制每次函数调用允许改变的最大速度。
     *
     * 所有正转、反转和停止过程都会经过这个限制，
     * 不再存在换向时直接跳变到负速度的特殊分支。
     */
    delta = Encoder_ClampFloat(
        delta,
        -MAX_DELTA_PER_CALL,
        MAX_DELTA_PER_CALL);

    *filtered_speed += delta;

    /*
     * 清除接近零的小残余速度，
     * 防止停止时一直显示很小的浮点值。
     */
    if((fabsf(*filtered_speed) < SPEED_ZERO_EPSILON) &&
       (fabsf(raw_speed) < SPEED_ZERO_EPSILON))
    {
        *filtered_speed = 0.0f;
    }

    return *filtered_speed;
}


/**
 * @brief 处理右编码器的一次 A 相中断
 *
 * 右编码器方向规则保持原代码不变：
 *
 *   A、B 电平不同：正转
 *   A、B 电平相同：反转
 */
static void Encoder_ProcessRightPulse(void)
{
    uint64_t pulse_time;
    uint32_t a;
    uint32_t b;
    bool a_high;
    bool b_high;
    int8_t observed_direction;

    pulse_time = mspm0_get_clock_us_now();

    /*
     * 软件最小脉冲间隔过滤。
     *
     * 这里只过滤时间明显过短的脉冲。
     * 不要把 MIN_PULSE_US 设置得过大，
     * 否则高速时可能删除正常编码器脉冲。
     */
    if((t0_R != 0U) &&
       ((pulse_time - t0_R) < MIN_PULSE_US))
    {
        return;
    }

    /* 读取右编码器 A、B 相当前电平 */
    a = DL_GPIO_readPins(
        GPIO_EncoderA_PORT,
        GPIO_EncoderA_PIN_0_PIN);

    b = DL_GPIO_readPins(
        GPIO_EncoderA_PORT,
        GPIO_EncoderA_PIN_1_PIN);

    /*
     * DL_GPIO_readPins() 返回的是引脚位掩码，
     * 所以必须先转换为 true/false，再进行逻辑比较。
     */
    a_high = (a != 0U);
    b_high = (b != 0U);

    /*
     * 保持原来的右编码器方向定义：
     *
     * A、B 不同：正转
     * A、B 相同：反转
     */
    if(a_high != b_high)
    {
        observed_direction = 1;
    }
    else
    {
        observed_direction = -1;
    }

    /*
     * 方向没有通过连续脉冲确认时：
     *
     * 1. 不修改 encoder_R；
     * 2. 不修改 t0_R、t1_R；
     * 3. 不修改稳定方向；
     * 4. 本次脉冲直接丢弃。
     */
    if(!Encoder_ConfirmDirection(
            &direction_R,
            &candidate_direction_R,
            &candidate_count_R,
            observed_direction))
    {
        return;
    }

    /*
     * 方向确认后，才允许更新时间戳。
     *
     * 这样异常方向脉冲不会污染速度周期。
     */
    t1_R = t0_R;
    t0_R = pulse_time;

    /* 更新右编码器累计计数 */
    if(direction_R > 0)
    {
        encoder_R++;
    }
    else
    {
        encoder_R--;
    }
}


/**
 * @brief 处理左编码器的一次 A 相中断
 *
 * 左电机与右电机镜像安装，因此方向规则与右编码器相反：
 *
 *   A、B 电平不同：反转
 *   A、B 电平相同：正转
 */
static void Encoder_ProcessLeftPulse(void)
{
    uint64_t pulse_time;
    uint32_t a;
    uint32_t b;
    bool a_high;
    bool b_high;
    int8_t observed_direction;

    pulse_time = mspm0_get_clock_us_now();

    /* 软件最小脉冲间隔过滤 */
    if((t0_L != 0U) &&
       ((pulse_time - t0_L) < MIN_PULSE_US))
    {
        return;
    }

    /* 读取左编码器 A、B 相当前电平 */
    a = DL_GPIO_readPins(
        GPIO_EncoderB_PORT,
        GPIO_EncoderB_PIN_2_PIN);

    b = DL_GPIO_readPins(
        GPIO_EncoderB_PORT,
        GPIO_EncoderB_PIN_3_PIN);

    a_high = (a != 0U);
    b_high = (b != 0U);

    /*
     * 左编码器与右编码器镜像安装，所以方向取反：
     *
     * A、B 不同：反转
     * A、B 相同：正转
     */
    if(a_high != b_high)
    {
        observed_direction = -1;
    }
    else
    {
        observed_direction = 1;
    }

    /* 候选换向没有确认时，本次脉冲不参与计数和测速 */
    if(!Encoder_ConfirmDirection(
            &direction_L,
            &candidate_direction_L,
            &candidate_count_L,
            observed_direction))
    {
        return;
    }

    /* 方向确认之后才更新时间戳 */
    t1_L = t0_L;
    t0_L = pulse_time;

    /* 更新左编码器累计计数 */
    if(direction_L > 0)
    {
        encoder_L++;
    }
    else
    {
        encoder_L--;
    }
}


/*==============================================================================
 *                              对外接口函数
 *============================================================================*/

/**
 * @brief 初始化左右编码器
 *
 * 1. 开启左右编码器 GPIO 中断；
 * 2. 启动提供微秒时间戳的硬件计时器。
 */
void Encoder_Init(void)
{
    /*
     * 清除编码器的软件状态。
     *
     * 如果程序支持运行过程中重新调用 Encoder_Init()，
     * 重新初始化后不会保留上一次运行的速度和方向。
     */
    __disable_irq();

    encoder_R = 0;
    encoder_L = 0;

    direction_R = 0;
    direction_L = 0;

    candidate_direction_R = 0;
    candidate_direction_L = 0;

    candidate_count_R = 0;
    candidate_count_L = 0;

    t0_R = 0;
    t1_R = 0;

    t0_L = 0;
    t1_L = 0;

    __enable_irq();

    speed_R_filt = 0.0f;
    speed_L_filt = 0.0f;

    /* 开启右编码器 GPIO 中断 */
    NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOA_INT_IRQN);

    /* 开启左编码器 GPIO 中断 */
    NVIC_EnableIRQ(GPIO_EncoderB_INT_IRQN);

    /*
     * 启动提供微秒时间戳的计时器。
     *
     * mspm0_get_clock_us_now() 应基于该计时器工作。
     */
    DL_Timer_startCounter(PWM_0_INST);
}


/**
 * @brief 编码器 GPIO 中断处理函数
 *
 * 该函数会同时检查左右编码器的中断状态。
 *
 * 外部真正的 GPIO IRQHandler 中应调用本函数。
 */
void Encodering(void)
{
    uint32_t encoder_R_port;
    uint32_t encoder_L_port;

    /*
     * 读取左右编码器中断状态。
     *
     * getEnabledInterruptStatus() 只返回已经使能的中断标志。
     */
    encoder_R_port =
        DL_GPIO_getEnabledInterruptStatus(
            GPIO_EncoderA_PORT,
            GPIO_EncoderA_PIN_0_PIN);

    encoder_L_port =
        DL_GPIO_getEnabledInterruptStatus(
            GPIO_EncoderB_PORT,
            GPIO_EncoderB_PIN_2_PIN);

    /* 处理右编码器 A 相中断 */
    if((encoder_R_port & GPIO_EncoderA_PIN_0_PIN) ==
       GPIO_EncoderA_PIN_0_PIN)
    {
        Encoder_ProcessRightPulse();
    }

    /*
     * 及时清除右编码器中断标志。
     *
     * 即使本次脉冲因为时间过短或者方向未确认而被丢弃，
     * GPIO 中断标志也必须清除。
     */
    DL_GPIO_clearInterruptStatus(
        GPIO_EncoderA_PORT,
        GPIO_EncoderA_PIN_0_PIN);

    /* 处理左编码器 A 相中断 */
    if((encoder_L_port & GPIO_EncoderB_PIN_2_PIN) ==
       GPIO_EncoderB_PIN_2_PIN)
    {
        Encoder_ProcessLeftPulse();
    }

    /* 清除左编码器中断标志 */
    DL_GPIO_clearInterruptStatus(
        GPIO_EncoderB_PORT,
        GPIO_EncoderB_PIN_2_PIN);
}


/**
 * @brief 获取右电机输出轴角速度
 *
 * @return 右电机滤波后的角速度，单位 rad/s
 *
 * 正数表示正转，负数表示反转。
 */
float GetSpeed_R(void)
{
    int8_t direction_copy;
    uint64_t t0_copy;
    uint64_t t1_copy;
    uint64_t current_time;

    /*
     * direction 和时间戳会在中断中修改。
     *
     * MSPM0 是 32 位 MCU，读取 uint64_t 不是原子操作，
     * 因此复制时间戳时必须暂时关闭中断。
     */
    __disable_irq();

    direction_copy = direction_R;
    t0_copy = t0_R;
    t1_copy = t1_R;

    __enable_irq();

    current_time = mspm0_get_clock_us_now();

    return Encoder_CalculateSpeed(
        direction_copy,
        t0_copy,
        t1_copy,
        current_time,
        &speed_R_filt);
}


/**
 * @brief 获取左电机输出轴角速度
 *
 * @return 左电机滤波后的角速度，单位 rad/s
 *
 * 正数表示正转，负数表示反转。
 */
float GetSpeed_L(void)
{
    int8_t direction_copy;
    uint64_t t0_copy;
    uint64_t t1_copy;
    uint64_t current_time;

    /* 原子复制中断中会更新的变量 */
    __disable_irq();

    direction_copy = direction_L;
    t0_copy = t0_L;
    t1_copy = t1_L;

    __enable_irq();

    current_time = mspm0_get_clock_us_now();

    return Encoder_CalculateSpeed(
        direction_copy,
        t0_copy,
        t1_copy,
        current_time,
        &speed_L_filt);
}