#include "key.h"
#include "clock.h"
#include "main.h"
#include "sys.h"


/**
 * @brief 按键扫描函数
 * 
 * @return uint8_t 按键编号（1-3），无按键按下返回0
 */
uint8_t Key_GetNum(void)
{
	uint8_t KeyNum = 0;
	
	if (DL_GPIO_readPins(KEY1_PORT, KEY1_KEY_1_PIN) == 0)
	{
		mspm0_delay_ms(20);
		while (DL_GPIO_readPins(KEY1_PORT, KEY1_KEY_1_PIN) == 0);
		mspm0_delay_ms(20);
		KeyNum = 1;
	}
	if (DL_GPIO_readPins(KEY2_PORT, KEY2_KEY_2_PIN) == 0)
	{
		mspm0_delay_ms(20);
		while (DL_GPIO_readPins(KEY2_PORT, KEY2_KEY_2_PIN) == 0);
		mspm0_delay_ms(20);
		KeyNum = 2;
	}
	if (DL_GPIO_readPins(KEY3_PORT, KEY3_KEY_3_PIN) == 0)
	{
		mspm0_delay_ms(20);
		while (DL_GPIO_readPins(KEY3_PORT, KEY3_KEY_3_PIN) == 0);
		mspm0_delay_ms(20);
		KeyNum = 3;
	}
	return KeyNum;
}

/**
 * @brief 按键工作函数
 */
void key_work(void) 
{
    key.keynum = Key_GetNum();
    if(key.keynum==1){key.keyspeed++;if(key.keyspeed>3)key.keyspeed=0;}
	if(key.keynum==2){key.start_flag = 1;}
    if(key.keynum==3){key.keymode++;if(key.keymode>4)key.keymode=0;}
}
