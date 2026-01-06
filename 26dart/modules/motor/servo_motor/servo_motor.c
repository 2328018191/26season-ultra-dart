#include "servo_motor.h"
#include "stdlib.h"
#include "string.h"
#include "Robot_cmd.h"
#include "cmsis_os.h"
extern TIM_HandleTypeDef htim8;
static ServoInstance *servo_motor_instance[SERVO_MOTOR_CNT] = {NULL};
static uint8_t servo_idx = 0;
const  RC_ctrl_t *rc_data;

// 通过此函数注册一个舵机
ServoInstance *ServoInit(Servo_Init_Config_s *Servo_Init_Config)
{
    ServoInstance *servo = (ServoInstance *)malloc(sizeof(ServoInstance));
    memset(servo, 0, sizeof(ServoInstance));
    servo->htim = Servo_Init_Config->htim;
    servo->Channel = Servo_Init_Config->Channel;
    HAL_TIM_PWM_Start(Servo_Init_Config->htim, Servo_Init_Config->Channel);
    
    servo_motor_instance[servo_idx++] = servo;
    
    while(rc_data == NULL)
    {
        osDelay(10);
        rc_data = get_rc_point();

    }
    return servo;

}









uint32_t UPServo_pwm;
uint32_t DownServo_pwm;
uint32_t EndServo_pwm;
uint32_t InitServo_pwm;
void RCdataToPWM(){
    //左竖直杆值对应大臂舵机
    InitServo_pwm= (uint32_t)((float)rc_data[TEMP].rc.rocker_l_ - RC_MIN_VALUE) * (max_pulse - min_pulse) 
                      / (RC_MAX_VALUE - RC_MIN_VALUE) + min_pulse;
    UPServo_pwm = (uint32_t)((float)rc_data[TEMP].rc.rocker_l1 - RC_MIN_VALUE) * (max_pulse - min_pulse) 
                      / (RC_MAX_VALUE - RC_MIN_VALUE) + min_pulse;
     DownServo_pwm = (uint32_t)((float)rc_data[TEMP].rc.rocker_r1 - RC_MIN_VALUE) * (max_pulse - min_pulse) 
                      / (RC_MAX_VALUE - RC_MIN_VALUE) + min_pulse;
     EndServo_pwm = (uint32_t)((float)rc_data[TEMP].rc.rocker_r_ - RC_MIN_VALUE) * (max_pulse - min_pulse) 
                      / (RC_MAX_VALUE - RC_MIN_VALUE) + min_pulse;

    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,InitServo_pwm);
    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,UPServo_pwm);
    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,DownServo_pwm);
    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,EndServo_pwm); 


}

//释放动作每次放完后就是这个动作
void InitPos(){

    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,1790);
    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,904);
    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,1750);
    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,1500);
}

void Release(){
    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,900);


}

void Grab(){
    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,1500);


}
//第一阶段就一直夹取
void FirstPos(){

    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,1910);
    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,904);
    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,1700);
    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,1500);}

    //第一阶段就一直夹取
void First(){

    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,1974);
    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,1044);
    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,1700);
    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,1500);}


//运动到第二个动作夹取第二个
void secondPos(){

    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,1000);
    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,1000);
    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,2000);
    osDelay(1000);
    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,1500);}

//运动到第三个动作夹取第二个
void thirdPos(){

    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,1800);
    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,1800);
    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,1200);
    osDelay(100);
    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,1500);}

