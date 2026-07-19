#include "sensor.h"
#include "motor.h"
#include "stdint.h"
#include "main.h"
#include "sys.h"
#include "sensor2.h"


static volatile int32_t   sensor_err2;

#define threshold 0 //阈值检测

static volatile int16_t motor_different_pid = 0, encoder_left_pid = 0, encoder_right_pid = 0;  // PID计算输出值

// PID静态变量，供复位使用
static float PID_Bias, PID_Integral_bias, PID_Pwm, PID_Last_Bias;

/**
 * @brief 复位差速PID状态，转弯退出时调用，清除积分残留
 */
void Motor_PID_Reset(void)
{
    PID_Bias = 0;
    PID_Integral_bias = 0;
    PID_Pwm = 0;
    PID_Last_Bias = 0;
    motor_different_pid = 0;
    encoder_left_pid = 0;
    encoder_right_pid = 0;
}

/**
 * @brief 差速位置模式PID控制器
 * 
 * @param Encoder 实际位置偏差
 * @param Target 目标值（通常为0）
 * @return int16_t 差速PWM输出值
 */
int16_t Motor_Different_Position_PID(int Encoder, int Target)
{
    float Position_KP =200, Position_KI = 0.4, Position_KD =365;
    
    PID_Bias = Encoder - Target;
    PID_Integral_bias += PID_Bias;
    if(PID_Bias == 0) PID_Integral_bias = 0;
    
    PID_Pwm = Position_KP * PID_Bias + Position_KI * PID_Integral_bias + Position_KD * (PID_Bias - PID_Last_Bias);
	    PID_Last_Bias = PID_Bias;

	    if(PID_Pwm > 3200) PID_Pwm = 3200;
        if(PID_Pwm < -3200) PID_Pwm = -3200;    
        
	    return PID_Pwm;
}


/**
 * @brief 位置环输出转换为速度环目标
 */
void Position_Adjust(void)
{
    // 速度阈值检测
    float omega_L = (base_speed - encoder_left_pid);
    if(omega_L >= threshold || omega_L <= -threshold)
    {
        //App_Motor_SetOmega_L(omega_L);
        App_PWM_Set_L(omega_L);
    }
    else
    {
        App_Motor_SetOmega_L(0);
    }

    float omega_R = (base_speed + encoder_right_pid);
    if(omega_R >= threshold || omega_R <= -threshold)
    {
        //App_Motor_SetOmega_R(omega_R);
        App_PWM_Set_R(omega_R);
    }
    else
    {
        App_Motor_SetOmega_R(0);
    }
}


void xunji_Proc(void)
{
        //获取灰度传感器数据
        //Get_error();
        Get_err2();

        int16_t motor_left_pid = 0, motor_right_pid = 0;

        // 计算基础速度和差速修正

        //motor_different_pid = Motor_Different_Position_PID(sensor_err, 0);

        sensor_err2 = Err2();
        motor_different_pid = Motor_Different_Position_PID(sensor_err2, 0);
        
        // 差速限幅，防止转向过度
        if(motor_different_pid > 3200) motor_different_pid = 3200;
        if(motor_different_pid < -3200) motor_different_pid = -3200;
        
        // 计算左右电机目标速度
        motor_left_pid = motor_different_pid;         // 左电机： 差速修正
        motor_right_pid = motor_different_pid;        // 右电机： 差速修正

        // 将电机速度映射到-100~+100范围
        // 基础速度约为1000对应直线速度100，转弯时速度会超过100但被限幅
        encoder_right_pid = (int16_t)(motor_right_pid * 100.0f / 3200.0f);
        encoder_left_pid = (int16_t)(motor_left_pid * 100.0f / 3200.0f);        

        // 更新速度环目标c
        Position_Adjust();

}
