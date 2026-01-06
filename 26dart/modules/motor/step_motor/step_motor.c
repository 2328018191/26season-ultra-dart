#include "step_motor.h"
#include "stdlib.h"
#include "string.h"
#include "vofa.h"
#include "cmsis_os.h"
#include <stdio.h>

// �ض���printf�����ڣ�����USART1��
// int __io_putchar(int ch) {
//     HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
//     return ch;
// }

#define STEPS_PER_REVOLUTION 200.0f // 360�� / 1.8�� = 200��
#define MICROSTEPPING 16.0f         // ���ʹ��΢������������΢�����ı���
#define DEGREES_PER_STEP (360.0f / (STEPS_PER_REVOLUTION * MICROSTEPPING))
#define daocheng 0.5f                                                   // 5mm��˿�˵���
#define radius 0.8f                                                     // 8mm��˿�˰뾶
#define round_radius 8.0f                                               // 80mm��ת���뾶
#define delta_pulse (daocheng / (STEPS_PER_REVOLUTION * MICROSTEPPING)) // ת��5mm���̵�ÿһ����5/3200
#define delta_angle (delta_pulse * 180.0f / (PI * radius))              // ת��5mm���̵�ÿһ����/����
// ��ֵת��Ϊ�������ת���Ƕ�,�������ת���Ƕ�����˿�˵��̶�Ӧ

static StepInstance *step_motor;
extern TIM_HandleTypeDef htim1;
static StepInstance *step_motor_instance[STEP_MOTOR_CNT] = {NULL};
static uint8_t step_idx = 0;
// ΢��ָ����ÿ�����������Ķ���//pulse/rev 3200
StepInstance *StepInit(Step_Init_Config_s *Step_Init_Config)
{

  StepInstance *step = (StepInstance *)malloc(sizeof(StepInstance));
  if (step == NULL)
  {
    // �����ڴ����ʧ�ܵ����
    return NULL;
  }
  memset(step, 0, sizeof(StepInstance));
  step->htim = Step_Init_Config->htim;
  step->Channel = Step_Init_Config->Channel;
  HAL_TIM_PWM_Start_IT(Step_Init_Config->htim, Step_Init_Config->Channel);
  step_motor_instance[step_idx++] = step;

  return step;
}

fp32 loop_fp32_constrain(fp32 Input, fp32 minValue, fp32 maxValue)
{
  if (maxValue < minValue)
  {
    return Input;
  }

  if (Input > maxValue)
  {
    fp32 len = maxValue - minValue;
    while (Input > maxValue)
    {
      Input -= len;
    }
  }
  else if (Input < minValue)
  {
    fp32 len = maxValue - minValue;
    while (Input < minValue)
    {
      Input += len;
    }
  }
  return Input;
}
#define rad_format(Ang) loop_fp32_constrain((Ang), -PI, PI)

// //
// // ��ֵת��Ϊ�������ת���Ƕ�,�������ת���Ƕ�����˿�˵��̶�Ӧ

void Step_Motor_Control_Init()
{
  __HAL_TIM_SetAutoreload(&htim1, 29999);             // ���õ�Ƶ�Ի�ýϴ������Ť��
  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 14999); // �����еȵ�ռ�ձȣ��ӽ�50%��
  HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_2);        // ����PWM

  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 14999); // �����еȵ�ռ�ձȣ��ӽ�50%��
  HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_4);        // ����PWM
}

void StepMotorStop()
{
  HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_4);
}
float deadband = 0.001f; // ������Χ

void StepMotorRotateDegrees(){
  float delta_yaw =0;    // ����������Ŀ��yaw-��ǰyaw
    	  htim1.Init.Period -= 468; // �����ø�Ƶ��Ť���𽥼���
  	 __HAL_TIM_SetAutoreload(&htim1, htim1.Init.Period);
 	  if (htim1.Init.Period < 468)
    {

      __HAL_TIM_SetAutoreload(&htim1, 468);
      __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2,234);  // 50%ռ�ձ�
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, (delta_yaw >= 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
      if(fabsf(delta_yaw)<deadband)

        StepMotorStop();
    }
    printf("delta_yaw:%f\r\n",delta_yaw);
}


void Step_Motor_Control_right()

{
  htim1.Init.Period -= 468;
  __HAL_TIM_SetAutoreload(&htim1, htim1.Init.Period);
  if (htim1.Init.Period < 468)
  {
    __HAL_TIM_SetAutoreload(&htim1, 468);             // ����������������
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 234); // ������������ռ�ձ�
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);
  }

  // ���Ʋ����������
}

void Step_Motor_Control_left()
{

  {
    htim1.Init.Period -= 468;
    __HAL_TIM_SetAutoreload(&htim1, htim1.Init.Period);
    if (htim1.Init.Period < 468)
    {
      __HAL_TIM_SetAutoreload(&htim1, 468);             // ����������������
      __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 234); // ������������ռ�ձ�
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);
    }
  }
}