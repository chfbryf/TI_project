#include "control.h"
#include "motor.h"
#include "sys.h"
#include "sensor.h"
#include "gyro.h"
#include "clock.h"

volatile uint32_t B0,H0;
volatile uint8_t B_H_flag,H_B_flag;
volatile uint32_t renwu_flag;

void renwu_reset(void)
{
    B0 = 0;
    H0 = 0;
    B_H_flag = 0;
    H_B_flag = 0;
}

void renwu1(void)
{
    mode = 2;
    if(L2 || L1 || M || R1 || R2)
    {
        if(delay_flag >= renwu_flag)
        {
            renwu_flag += 10;
            if(L2 || L1 || M || R1 || R2)
            {
                mode = 1;
            }
        }
    }
}

void renwu2(void)
{
    if(L2 || L1 || M || R1 || R2)
    {
        if(delay_flag >= renwu_flag)
        {
            renwu_flag += 10;
            if(L2 || L1 || M || R1 || R2)
            {
                mode = 3;
                H_B_flag = 1;
                if(B_H_flag == 1){
                    H0++;
                    B_H_flag = 0;
                }
            }
        }
    }
    else {
        mode = 2;

        B_H_flag = 1;
        if(H_B_flag == 1){
            B0++;
            H_B_flag = 0;
        }
    }
    if(B0 >= 2 && H0 >= 2)
    {
        mode = 1;
    }
}

void renwu3(void)
{
    if(L2 || L1 || M || R1 || R2)
    {
        if(delay_flag >= renwu_flag)
        {
            renwu_flag += 10;
            if(L2 || L1 || M || R1 || R2)
            {
                mode = 3;
                H_B_flag = 1;
                if(B_H_flag == 1){
                    H0++;
                    B_H_flag = 0;
                }
            }
        }
    }
    else {
       B_H_flag = 1;
        mode = 4;
        if(H_B_flag == 1){
            B0++;
            H_B_flag = 0;
        }

        if(B0%2 == 0)
        {
            omega_flag = 0;
        }
        else
        {
            omega_flag = 1;
        }

    }
    if(B0 >= 2 && H0 >= 2)
    {
        mode = 1;
    }

}

void renwu4(void)
{
    if(L2 || L1 || M || R1 || R2)
    {
        if(delay_flag >= renwu_flag)
        {
            renwu_flag += 10;
            if(L2 || L1 || M || R1 || R2)
            {
                mode = 3;
                H_B_flag = 1;
                if(B_H_flag == 1){
                    H0++;
                    B_H_flag = 0;
                }
            }
        }
    }
    else {
        B_H_flag = 1;
        mode = 4;
        if(H_B_flag == 1){
            B0++;
            H_B_flag = 0;
        }

        if(B0%2 == 0)
        {
            omega_flag = 0;
        }
        else
        {
            omega_flag = 1;
        }

    }
    if(B0 >= 8 && H0 >= 8)
    {
        mode = 1;
    }

}

