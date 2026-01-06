#ifndef SERVO_MOTOR_H
#define SERVO_MOTOR_H

#include <stdint.h>
#include "main.h"
#include "tim.h"
#include "remote_control.h"
#define SERVO_MOTOR_CNT 7
#define max_pulse 2500 //控制周期2.5ms对应270度
#define min_pulse 500 //0
#define middle_pulse 1500   //135
#define min_control 3   //最小控制精度3us对应的最小控制角度
#define RC_MIN_VALUE -500
#define RC_MAX_VALUE 500
#define LAST 1
#define TEMP 0
// 外部配置舵机所需实例，只是赋予外部值，相当于中间变量给参数
typedef struct
{

    // 使用的定时器类型及通道
    TIM_HandleTypeDef *htim;
    uint32_t Channel;


} Servo_Init_Config_s;

// 存储好的步进电机实例，对此进行操作
typedef struct
{
    TIM_HandleTypeDef *htim; // 选择的定时器
    uint32_t Channel;        // 选择的定时器通道
} ServoInstance;

ServoInstance *ServoInit(Servo_Init_Config_s *Servo_Init_Config);
void RCdataToPWM();
void InitPos();
void secondPos();
void FirstPos();
void First();
void thirdPos();
void Release();
void Grab();
#endif