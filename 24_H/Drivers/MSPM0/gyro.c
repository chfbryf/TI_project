#include "gyro.h"
#include "main.h"
#include "motor.h"
#include "sys.h"


static int16_t motor_gyro_different_pid = 0, gyro_left_pid = 0, gyro_right_pid = 0;  // PID计算输出值


#define threshold 0.1  // 阈值检测

// 角度环PID
#define   Kp       10
#define   Ki       0
#define   Kd  	   0




float GYRO_Control(float now,float target)
{
	static float Bias, Last_bias, Last2_bias, Pwm;
	Bias = target-now;
	Pwm += Kp * (Bias - Last_bias) + Ki * Bias + Kd * (Bias - 2 * Last_bias + Last2_bias);

	    Last2_bias = Last_bias;
		Last_bias = Bias;

	if(Pwm > 1000) Pwm = 1000;
    if(Pwm < -1000) Pwm = -1000;
	return Pwm;
}


/**
 * @brief 位置环输出转换为速度环目标
 */
void GYRO_Adjust(void)
{
    float omega_L = base_speed - gyro_left_pid;
    if(omega_L >= threshold || omega_L <= -threshold)
    {
        App_Motor_SetOmega_L(omega_L);
    }
    else
    {
        App_Motor_SetOmega_L(0);
    }

    float omega_R = base_speed + gyro_right_pid;
    if(omega_R >= threshold || omega_R <= -threshold)
    {
        App_Motor_SetOmega_R(omega_R);
    }
    else
    {
        App_Motor_SetOmega_R(0);
    }
}


void GYRO_Proc(float target_yaw)
{

        int16_t motor_left_pid = 0, motor_right_pid = 0;

        // 计算基础速度和差速修正

        motor_gyro_different_pid = GYRO_Control(yaw, target_yaw);
        
        // 差速限幅，防止转向过度
        if(motor_gyro_different_pid > 1000) motor_gyro_different_pid = 1000;
        if(motor_gyro_different_pid < -1000) motor_gyro_different_pid = -1000;
        
        // 计算左右电机目标速度
        motor_left_pid = motor_gyro_different_pid;         // 左电机： 差速修正
        motor_right_pid = motor_gyro_different_pid;        // 右电机： 差速修正

        // 将电机速度映射到-100~+100范围
        // 基础速度约为1000对应直线速度100，转弯时速度会超过100但被限幅
        gyro_right_pid = (int16_t)(motor_right_pid * 100.0f / 1000.0f);
        gyro_left_pid = (int16_t)(motor_left_pid * 100.0f / 1000.0f);        

        // 更新速度环目标
        GYRO_Adjust();

}

