#include "ti_msp_dl_config.h"

#define AIN1_High       DL_GPIO_setPins(DIR_AIN1_PORT, DIR_AIN1_PIN)
#define AIN1_Low        DL_GPIO_clearPins(DIR_AIN1_PORT, DIR_AIN1_PIN)
#define AIN2_High       DL_GPIO_setPins(DIR_AIN2_PORT, DIR_AIN2_PIN)
#define AIN2_Low        DL_GPIO_clearPins(DIR_AIN2_PORT, DIR_AIN2_PIN)
#define BIN1_High       DL_GPIO_setPins(DIR_BIN1_PORT, DIR_BIN1_PIN)
#define BIN1_Low        DL_GPIO_clearPins(DIR_BIN1_PORT, DIR_BIN1_PIN)
#define BIN2_High       DL_GPIO_setPins(DIR_BIN2_PORT, DIR_BIN2_PIN)
#define BIN2_Low        DL_GPIO_clearPins(DIR_BIN2_PORT, DIR_BIN2_PIN)


#define LED3_High        DL_GPIO_setPins(LED_PORT, LED_LED3_PIN)
#define LED3_Low         DL_GPIO_clearPins(LED_PORT, LED_LED3_PIN)
#define LED4_High        DL_GPIO_setPins(LED_PORT, LED_LED4_PIN)
#define LED4_Low         DL_GPIO_clearPins(LED_PORT, LED_LED4_PIN)

#define BUZZ_High        DL_GPIO_setPins(BUZZER_PORT, BUZZER_buzzer_PIN)
#define BUZZ_Low         DL_GPIO_clearPins(BUZZER_PORT, BUZZER_buzzer_PIN)

/* 跨模块全局变量声明 */
extern volatile uint8_t  keyspeed;
extern volatile uint8_t  keyquan;
extern volatile uint8_t  keynum;
extern volatile int16_t  base_speed;
extern volatile uint8_t  start_flag;
