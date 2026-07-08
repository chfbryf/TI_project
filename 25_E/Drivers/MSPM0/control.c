#include "control.h"
#include "motor.h"
#include "sensor.h"
#include "stdint.h"
#include "main.h"
#include "gyro.h"
#include "sys.h"



static volatile int16_t motor_different_pid, encoder_left_pid, encoder_right_pid;  // PID计算输出值
static volatile int32_t Position_err = 0;

/**
 * @brief 差速位置模式PID控制器
 * 
 * @param Encoder 实际位置偏差
 * @param Target 目标值（通常为0）
 * @return int16_t 差速PWM输出值
 */
int16_t Motor_Different_Position_PID(int Encoder, int Target)
{
    float Position_KP = 3, Position_KI = 0, Position_KD = 0;
    static float Bias, Integral_bias, Pwm, Last_Bias;
    
    Bias = Encoder - Target;
    Integral_bias += Bias;
    if(Bias == 0) Integral_bias = 0;
    
    Pwm = Position_KP * Bias + Position_KI * Integral_bias + Position_KD * (Bias - Last_Bias);
    Last_Bias = Bias;
    
    return Pwm;
}


/**
 * @brief 位置环输出转换为速度环目标
 */
void Position_Adjust(void)
{
    // 速度阈值检测：仅当基础速度大于2时才进入速度环
    if(encoder_left_pid >= 5)
    {
        float omega_L = base_speed - encoder_left_pid; 

    
        App_Motor_SetOmega_L(omega_L);
    }
    else
    {
        App_Motor_SetOmega_L(0);

    }

    if(encoder_right_pid <= 2) {
        App_Motor_SetOmega_R(0);
    }

    if(encoder_right_pid >= 5)
    {
        float omega_R = base_speed + encoder_right_pid; 

    
        App_Motor_SetOmega_R(omega_R);
    }
    else
    {
        App_Motor_SetOmega_R(0);

    }
}


void xunji_Proc(void)
{
        //获取灰度传感器数据
        Get_error();

        int16_t motor_left_pid = 0, motor_right_pid = 0;

        //获取误差
        Position_err = Error();


        // 计算基础速度和差速修正

        motor_different_pid = Motor_Different_Position_PID(Position_err, 0);
        
        // 差速限幅，防止转向过度
        if(motor_different_pid > 4000) motor_different_pid = 4000;
        if(motor_different_pid < -4000) motor_different_pid = -4000;
        
        // 计算左右电机目标速度
        motor_left_pid = motor_different_pid;         // 左电机： 差速修正
        motor_right_pid = motor_different_pid;        // 右电机： 差速修正

        // 将电机速度映射到-100~+100范围
        // 基础速度约为4000对应直线速度100，转弯时速度会超过100但被限幅
        encoder_right_pid = (int16_t)(motor_right_pid * 100.0f / 4000.0f);
        encoder_left_pid = (int16_t)(motor_left_pid * 100.0f / 4000.0f);        

        // 更新速度环目标
        Position_Adjust();

}
