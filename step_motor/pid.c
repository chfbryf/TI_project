#include "pid.h"

/**
 * @brief 初始化 PID 控制器
 * @param pid           PID 结构体指针
 * @param kp            比例系数
 * @param ki            积分系数
 * @param kd            微分系数
 * @param setpoint      设定目标值
 * @param output_limit  输出限幅（绝对值）
 */
void PID_Init(PID_Controller *pid, float kp, float ki, float kd, float setpoint, float output_limit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->setpoint = setpoint;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output_limit = output_limit;
    pid->last_time = 0; // 首次调用时会自动计算 dt
}

/**
 * @brief 进行一次 PID 运算，返回控制输出
 * @param pid       PID 结构体指针
 * @param feedback  当前反馈值（测量值）
 * @param dt        距离上次调用的时间间隔（单位：秒）
 * @return          控制输出（角速度增量，或直接角度增量）
 */
float PID_Compute(PID_Controller *pid, float feedback, float dt)
{
    // 防止除零
    if (dt < 0.0001f) dt = 0.001f;

    float error = pid->setpoint - feedback;

    // 积分项（梯形积分）
    pid->integral += error * dt;

    // 微分项（误差微分，避免微分冲击）
    float derivative = (error - pid->prev_error) / dt;

    // 计算输出
    float output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;

    // 输出限幅
    if (output > pid->output_limit)
        output = pid->output_limit;
    else if (output < -pid->output_limit)
        output = -pid->output_limit;

    // 积分抗饱和（clamping）：如果输出被限幅且误差方向与输出方向相同，则回退本次积分
    if ((output == pid->output_limit && error > 0) || (output == -pid->output_limit && error < 0))
    {
        pid->integral -= error * dt;  // 回退本次积分
    }

    pid->prev_error = error;
    return output;
}
