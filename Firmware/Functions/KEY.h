#ifndef _KEY_h
#define _KEY_h
#include "stdint.h"

/*  多余的按键可以删除掉，如要添加按键需要复制第7、13行，分别粘贴到第11、17行的下一行  */

#define KEY1    HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_10) //检测PB10，并把PB10命名为KEY1
#define KEY2    HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_11) //检测PB11，并把PB10命名为KEY2
#define KEY3    HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_12) //检测PB12，并把PB10命名为KEY3
#define KEY4    HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_13) //检测PB13，并把PB10命名为KEY4
#define KEY5    HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14) //检测PB14，并把PB10命名为KEY5

#define KEY1_PRES   1       //按键1按下，设置为数字1，并通过函数返回
#define KEY2_PRES   2       //按键2按下，设置为数字2，并通过函数返回
#define KEY3_PRES   3       //按键3按下，设置为数字3，并通过函数返回
#define KEY4_PRES   4       //按键4按下，设置为数字4，并通过函数返回
#define KEY5_PRES   5       //按键5按下，设置为数字5，并通过函数返回

uint8_t KEY_Scan(uint8_t mode);   //按键扫描函数，mode:0,不支持连续按;1,支持连续按;

#endif
