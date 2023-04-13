#pragma once

#include <driver/gpio.h>


//OLD
// #define GPIO_LCD_RESET  GPIO_NUM_1
// #define GPIO_LCD_SDA    GPIO_NUM_2
// #define GPIO_IO_OFF     GPIO_NUM_3
// #define GPIO_RGB        GPIO_NUM_4
// #define GPIO_BAT_ADC    GPIO_NUM_5
// #define GPIO_BAT_ADC_EN GPIO_NUM_6
// #define GPIO_PUSH_UP    GPIO_NUM_9
// #define GPIO_PUSH       GPIO_NUM_10
// #define GPIO_MOTOR_UH   GPIO_NUM_12
// #define GPIO_MOTOR_UL   GPIO_NUM_13
// #define GPIO_MOTOR_VH   GPIO_NUM_14
// #define GPIO_MOTOR_VL   GPIO_NUM_21
// #define GPIO_MOTOR_WL   GPIO_NUM_48
// #define GPIO_MOTOR_WH   GPIO_NUM_47
// #define GPIO_MOTOR_DIAG GPIO_NUM_35
// #define GPIO_MM_SDA     GPIO_NUM_36
// #define GPIO_MM_SCL     GPIO_NUM_37
// #define GPIO_MM_CS      GPIO_NUM_38
// #define GPIO_LCD_BL     GPIO_NUM_39
// #define GPIO_LCD_DC     GPIO_NUM_40
// #define GPIO_LCD_CS     GPIO_NUM_41
// #define GPIO_LCD_SCL    GPIO_NUM_42

//NEW
#define GPIO_IO_OFF     GPIO_NUM_3
#define GPIO_RGB        GPIO_NUM_4
#define GPIO_BAT_ADC    GPIO_NUM_5
#define GPIO_BAT_ADC_EN GPIO_NUM_6
#define GPIO_PUSH_UP    GPIO_NUM_9
#define GPIO_PUSH       GPIO_NUM_10
#define GPIO_MOTOR_UH   GPIO_NUM_21
#define GPIO_MOTOR_UL   GPIO_NUM_35
#define GPIO_MOTOR_VH   GPIO_NUM_47
#define GPIO_MOTOR_VL   GPIO_NUM_37
#define GPIO_MOTOR_WL   GPIO_NUM_36
#define GPIO_MOTOR_WH   GPIO_NUM_48
#define GPIO_MOTOR_DIAG GPIO_NUM_14
#define GPIO_MM_SDA     GPIO_NUM_13
#define GPIO_MM_SCL     GPIO_NUM_12
#define GPIO_MM_CS      -1
#define GPIO_LCD_BL     GPIO_NUM_39
#define GPIO_LCD_DC     GPIO_NUM_40
#define GPIO_LCD_CS     GPIO_NUM_41
#define GPIO_LCD_SCL    GPIO_NUM_42 
#define GPIO_LCD_RESET  GPIO_NUM_1
#define GPIO_LCD_SDA    GPIO_NUM_2