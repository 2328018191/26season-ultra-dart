#ifndef STEP_MOTOR_H
#define STEP_MOTOR_H

#include <stdint.h>
#include "main.h"
#include "tim.h"
#include "remote_control.h"
#include "visual.h"
#define STEP_MOTOR_CNT 7
// ����ң�������ݶ�ȡ,ң����������һ����СΪ2������
#define LAST 1
#define TEMP 0
typedef float fp32;
//�ⲿ���ò����������ʵ����ֻ�Ǹ����ⲿֵ���൱���м����������
typedef struct
{

    // ʹ�õĶ�ʱ�����ͼ�ͨ��
    TIM_HandleTypeDef *htim;
    uint16_t Channel;

} Step_Init_Config_s;

//�洢�õĲ������ʵ�����Դ˽��в���
typedef struct
{
    TIM_HandleTypeDef *htim; //ѡ��Ķ�ʱ��
    uint16_t Channel; //ѡ��Ķ�ʱ��ͨ��
} StepInstance;





StepInstance *StepInit(Step_Init_Config_s *Step_Init_Config);
void StepMotorStop();
void Step_Motor_Control_right();
void Step_Motor_Control_left();

void Pitch_Motor_Control_left();
void Pitch_Motor_Control_right();
void Step_Motor_Control_Init();
void StepMotorRotateDegrees();

void HAL_TIM_PeriodElapsedCallback();

#endif