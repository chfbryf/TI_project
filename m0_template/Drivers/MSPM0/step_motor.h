#ifndef STEP_MOTOR_H
#define STEP_MOTOR_H

#include "ti_msp_dl_config.h"

void step_motor_Init(void);
void step_motor_start(uint8_t stepper_id);
void step_motor_stop(uint8_t stepper_id);
void step_set_speed(float speed, uint8_t stepper_id);
void step_motor_dir_set(uint8_t direction, uint8_t stepper_id);
void step_rotate_by(float degrees, uint8_t direction, uint8_t stepper_id);

/**
 * @brief 连续调速模式（用于 PID 实时控制）
 * @param stepper_id  电机编号 (1 或 2)
 * @param speed_dps   角速度，单位 度/秒，正=正转，负=反转
 *
 * 与 test 的 Set_PWM(L_PWM,R_PWM) 思路一致：PID 输出直接控制转速。
 * speed_dps 绝对值 < 0.06 时自动停转（死区），避免微小误差导致电机啸叫。
 */
void step_motor_continuous_run(uint8_t stepper_id, float speed_dps);

#endif
