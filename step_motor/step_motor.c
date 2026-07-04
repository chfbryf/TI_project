#include "step_motor.h"

/*

PWM:脉冲信号，一个脉冲代表一个微步
DIR:方向控制
DCY:电流衰减；高电平大扭矩
SLP:休眠，高电平工作
RST:复位，高电平工作
GND：共地

一脉冲 0。05625度
角速度 = 0.05625度 * 脉冲频率
脉冲频率 = 0.05625度 * 角速度
30角速度 = 30 / 0.05625 = 533.33Hz

*/

uint32_t step_remain_1 = 0;
uint32_t step_remain_2 = 0;


//
//@简介：步进电机初始化
//
void step_motor_Init(void)
{
    DL_GPIO_setPins(STEP_MOTOR_PORT, STEP_MOTOR_RST2_PIN);
    DL_GPIO_setPins(STEP_MOTOR_PORT, STEP_MOTOR_SLP2_PIN);
    DL_GPIO_setPins(STEP_MOTOR_PORT, STEP_MOTOR_DIR2_PIN);
    DL_GPIO_setPins(STEP_MOTOR_PORT, STEP_MOTOR_DCY2_PIN);
    DL_GPIO_setPins(STEP_MOTOR_PORT, STEP_MOTOR_RST1_PIN);
    DL_GPIO_setPins(STEP_MOTOR_PORT, STEP_MOTOR_SLP1_PIN);
    DL_GPIO_setPins(STEP_MOTOR_PORT, STEP_MOTOR_DIR1_PIN);
    DL_GPIO_setPins(STEP_MOTOR_PORT, STEP_MOTOR_DCY1_PIN);

    NVIC_EnableIRQ(DCC_100_PWM1_INST_INT_IRQN);
    NVIC_EnableIRQ(DCC_100_PWM2_INST_INT_IRQN);


}



//
//@简介：方向控制
//@参数：direction-方向,0-低电平，1-高电平
//@参数：stepper-电机编号
//
void step_motor_dir_set(uint8_t direction, uint8_t stepper_id)
{
    if(stepper_id == 1)
    {
        if(direction == 0)
        {
            DL_GPIO_clearPins(STEP_MOTOR_PORT, STEP_MOTOR_DIR1_PIN);
        }
        else{
            DL_GPIO_setPins(STEP_MOTOR_PORT, STEP_MOTOR_DIR1_PIN);
        }
    }

    if(stepper_id == 2)
    {
        if(direction == 0)
        {
            DL_GPIO_clearPins(STEP_MOTOR_PORT, STEP_MOTOR_DIR2_PIN);
        }
        else{
            DL_GPIO_setPins(STEP_MOTOR_PORT, STEP_MOTOR_DIR2_PIN);
        }
    }
}


//
//@简介：步进电机初始化
//@参数：stepper-电机编号
//
void step_motor_start(uint8_t steeper_id) 
{
    if(steeper_id == 1)
    {
        NVIC_EnableIRQ(DCC_100_PWM1_INST_INT_IRQN);
        DL_Timer_startCounter(DCC_100_PWM1_INST);
    }

    if(steeper_id == 2)
    {
        NVIC_EnableIRQ(DCC_100_PWM2_INST_INT_IRQN);
        DL_Timer_startCounter(DCC_100_PWM2_INST);
    }
}


void step_motor_stop(uint8_t stepper_id)
{
    if(stepper_id == 1)
    {
    DL_Timer_stopCounter(DCC_100_PWM1_INST);
    DL_Timer_stopCounter(DCC_100_PWM1_INST);
    }

    if(stepper_id == 2)
    {
    DL_Timer_stopCounter(DCC_100_PWM2_INST);
    DL_Timer_stopCounter(DCC_100_PWM2_INST);
    }
}


//
//@简介：设置步进电机速度
//@参数：speed-速度，单位：Hz
//@参数：stepper-电机编号
//
void step_set_speed(uint16_t speed, uint8_t stepper_id)
{
    if(stepper_id == 1)
    {

        uint32_t frequency = (uint32_t)(speed / 0.05625);   // 计算脉冲频率
        frequency = frequency > 0 ? frequency : 1;
        //计算定时器溢出值
        uint32_t period = DCC_100_PWM1_INST_CLK_FREQ / frequency;
        //限幅
        period = period < 65535 ? period : 65535;
        period = period > 800 ? period :800;

        DL_Timer_setLoadValue(DCC_100_PWM1_INST, period);
        DL_Timer_setCaptureCompareValue(DCC_100_PWM1_INST, period / 2, GPIO_DCC_100_PWM1_C0_IDX);   //设置占空比为50%
    }


    if(stepper_id == 2)
    {

        uint32_t frequency = (uint32_t)(speed / 0.05625);   // 计算脉冲频率
        frequency = frequency > 0 ? frequency : 1;
        //计算定时器溢出值
        uint32_t period = DCC_100_PWM2_INST_CLK_FREQ / frequency;
        //限幅
        period = period < 65535 ? period : 65535;
        period = period > 800 ? period :800;

        DL_Timer_setLoadValue(DCC_100_PWM2_INST, period);
        DL_Timer_setCaptureCompareValue(DCC_100_PWM2_INST, period / 2, GPIO_DCC_100_PWM2_C0_IDX);   //设置占空比为50%
    }
}


void step_set_angle(uint8_t angle, uint8_t stepper_id)
{
    if(stepper_id == 1)
    {
        step_remain_1 = (uint32_t)(angle / 0.05625);   // 计算脉冲频率
        step_motor_start(stepper_id);
    }

    if(stepper_id == 2)
    {
        step_remain_2 = (uint32_t)(angle / 0.05625);   // 计算脉冲频率
        step_motor_start(stepper_id);
    }
}


void DCC_100_PWM2_INST_IRQHandler()
{
    switch (DL_Timer_getPendingInterrupt(DCC_100_PWM2_INST))
    {
        case DL_TIMER_IIDX_LOAD:
        {
            if(step_remain_2 == 0)
            {
                step_motor_stop(2);
            }
            step_remain_2 --;

            break;
        }

    default:
        break;
    }
}

    
void DCC_100_PWM1_INST_IRQHandler()
{
    switch (DL_Timer_getPendingInterrupt(DCC_100_PWM1_INST))
    {
        case DL_TIMER_IIDX_LOAD:
        {
            if(step_remain_1 == 0)
            {
                step_motor_stop(1);
            }
            step_remain_1 --;

            break;
        }

    default:
        break;
    }
}

