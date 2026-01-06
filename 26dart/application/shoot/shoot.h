#ifndef SHOOT_H
#define SHOOT_H

#include "main.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "message_center.h"
#include "usart.h"
#include "stm32f4xx_hal_uart.h"
// 39500 32500前哨战
// 46500 36500基地
#define high_speed 37350  // 摩擦轮的高速37250   36500      37500
#define low_speed 34050      // 摩擦轮的低速34000           
// #define high_speed 37500
// #define low_speed 34650

void Shoot_Init();
void Shoot_Task();
 


#endif