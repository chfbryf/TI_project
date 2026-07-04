#include "encoder.h"
#include "clock.h"

static volatile uint32_t Encoder_R_Port, Encoder_L_Port;
static volatile int64_t encoder_L = 0, encoder_R = 0;	//编码器的值
static volatile int8_t direction_L = 0;	//左编码器方向，1——正转， -1——反转
static volatile int8_t direction_R = 0;	//右编码器方向，1——正转， -1——反转
static volatile uint64_t t0_L = 0, t1_L = 0, t0_R = 0, t1_R = 0;	//编码器变化的时间， t0——当前时间， t1——上次时间


/**
 * @brief 编码器初始化
 */
void Encoder_Init(void)
{
    NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOA_INT_IRQN);
    NVIC_EnableIRQ(GPIO_EncoderB_INT_IRQN);

    DL_Timer_startCounter(PWM_0_INST);
}


/**
 * @brief 编码器中断读取函数
 */
void Encodering(void)
{

    Encoder_R_Port = DL_GPIO_getEnabledInterruptStatus(GPIO_EncoderA_PORT, GPIO_EncoderA_PIN_0_PIN);
    Encoder_L_Port = DL_GPIO_getEnabledInterruptStatus(GPIO_EncoderB_PORT, GPIO_EncoderB_PIN_2_PIN);
    
    /* 编码器A — 对应右电机 */
    if((Encoder_R_Port & GPIO_EncoderA_PIN_0_PIN) == GPIO_EncoderA_PIN_0_PIN)        //编码器A-Pin0
    {
        t1_R = t0_R;
		t0_R =  mspm0_get_clock_us_now();

        uint32_t a = DL_GPIO_readPins(GPIO_EncoderA_PORT, GPIO_EncoderA_PIN_0_PIN);	//获取A相当前电压
		uint32_t b = DL_GPIO_readPins(GPIO_EncoderA_PORT, GPIO_EncoderA_PIN_1_PIN);	//获取B相当前电压


		if((a == 0 && b != 0) || (a != 0 && b == 0))	//当前轮胎正转(A、B相异)
	{
		encoder_R++;

		if(direction_R < 0)	//之前轮胎反转
		{
			direction_R = 2;
		}
		else
		{
		direction_R = 1;
		}
	}

	else	//当前轮胎反转(A、B相同)
	{
		encoder_R--;

		if(direction_R > 0)	//之前轮胎正转
		{
			direction_R = -2;
		}
		else
		{
		direction_R = -1;
		}
	}
        
    }
    DL_GPIO_clearInterruptStatus(GPIO_EncoderA_PORT, GPIO_EncoderA_PIN_0_PIN);
 
 
    /* 编码器B — 对应左电机 */
    if((Encoder_L_Port & GPIO_EncoderB_PIN_2_PIN) == GPIO_EncoderB_PIN_2_PIN)        //编码器B-Pin2
    {
        t1_L = t0_L;
		t0_L =  mspm0_get_clock_us_now();

        uint32_t a = DL_GPIO_readPins(GPIO_EncoderB_PORT, GPIO_EncoderB_PIN_2_PIN);	//获取A相当前电压
		uint32_t b = DL_GPIO_readPins(GPIO_EncoderB_PORT, GPIO_EncoderB_PIN_3_PIN);	//获取B相当前电压


	if((a == 0 && b != 0) || (a != 0 && b == 0))	//当前轮胎反转(A、B相异)
	{
		encoder_L--;

		if(direction_L > 0)	//之前轮胎正转
		{
			direction_L = -2;
		}
		else
		{
		direction_L = -1;
		}

	}
	else	//当前轮胎正转(A、B相同)
	{

		encoder_L++;

		if(direction_L < 0)	//之前轮胎反转
		{
			direction_L = 2;
		}
		else
		{
		direction_L = 1;
		}

	}
        
    }
    DL_GPIO_clearInterruptStatus(GPIO_EncoderB_PORT, GPIO_EncoderB_PIN_2_PIN);
}


/**
 * @brief 读取右电机旋转角速度
 * 
 * now由软件触发，而t0是由硬件触发
 * 
 * @return float 右电机角速度，单位rad/s
 */
float GetSpeed_R(void)
{

	__disable_irq();//关闭单片机总中断

	//解决毛刺问题

	int8_t direction_cpy_R = direction_R;
	uint64_t t0_cpy = t0_R;
	uint64_t t1_cpy = t1_R;

	__enable_irq();//开启单片机总中断


	if(direction_cpy_R == 2 || direction_cpy_R == -2)	return 0.0f;//解决换向问题

	uint64_t now = mspm0_get_clock_us_now();//获取当前时间并解决不为0的问题
	float T;

	// 优先使用脉冲间周期(稳定值), 仅在长时间无脉冲时用 now-t0 防止速度悬停
	if(t0_cpy == t1_cpy || (now - t0_cpy) > (t0_cpy - t1_cpy) * 3)
	{
		T = (now - t0_cpy) * 1.0e-6f;
	}
	else
	{
		T = (t0_cpy - t1_cpy) * 1.0e-6f;
	}

	if(T < 1.0e-6f) T = 1.0e-6f;	//防止除零

	return direction_cpy_R / T / 22.0f / 30 * 6.2831853f;
}


/**
 * @brief 读取左电机旋转角速度
 * 
 * now由软件触发，而t0是由硬件触发
 * 
 * @return float 左电机角速度，单位rad/s
 */
float GetSpeed_L(void)
{

	__disable_irq();//关闭单片机总中断

	//解决毛刺问题

	int8_t direction_cpy_L = direction_L;
	uint64_t t0_cpy = t0_L;
	uint64_t t1_cpy = t1_L;

	__enable_irq();//开启单片机总中断


	if(direction_cpy_L == 2 || direction_cpy_L == -2)	return 0.0f;//解决换向问题

	uint64_t now = mspm0_get_clock_us_now();//获取当前时间并解决不为0的问题
	float T;

	// 优先使用脉冲间周期(稳定值), 仅在长时间无脉冲时用 now-t0 防止速度悬停
	if(t0_cpy == t1_cpy || (now - t0_cpy) > (t0_cpy - t1_cpy) * 3)
	{
		T = (now - t0_cpy) * 1.0e-6f;
	}
	else
	{
		T = (t0_cpy - t1_cpy) * 1.0e-6f;
	}

	if(T < 1.0e-6f) T = 1.0e-6f;	//防止除零

	return direction_cpy_L / T / 22.0f / 30 * 6.2831853f;
}

