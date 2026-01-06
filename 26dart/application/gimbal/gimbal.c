#include "gimbal.h"
#include "step_motor.h"


extern TIM_HandleTypeDef htim1;

static DJIMotorInstance *yaw_motor;

static Publisher_t *gimbal_pub;                   // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;                  // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息
static float init_yaw_angle = 0;

void Gimbal_Init()
{
    Motor_Init_Config_s yaw_motor_config = {
        .can_init_config.can_handle = &hcan1,
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 3.0, // 4.5
                .Ki = 2, // 0
                .Kd = 0, // 0
                .IntegralLimit = 5000,
                .Improve = PID_Integral_Limit,
                .MaxOut = 15000,
            },
            .current_PID = {
                .Kp = 0.6, // 0.4
                .Ki = 0, // 0
                .Kd = 0,
                .IntegralLimit = 10000,
                .Improve = PID_Integral_Limit,
                .MaxOut = 15000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP, // | CURRENT_LOOP,
        },
        .motor_type = M3508,
    };
    yaw_motor_config.can_init_config.tx_id = 2; // 中间扳机换弹3508
    yaw_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    yaw_motor = DJIMotorInit(&yaw_motor_config);


    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
}




void Gimbal_Task()
{


    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);
    switch (gimbal_cmd_recv.gimbal_mode)
    {
    // 停止，没有电流
    case GIMABL_FREE_MODE:
        DJIMotorStop(yaw_motor);

        break;

    // 正常由遥控器控制运动向右转动(右中)
    case GIMBAL_YAW_LEFT:
        DJIMotorEnable(yaw_motor);
        DJIMotorSetRef(2000);
        break;

 

    case GIMBAL_YAW_RIGHT://上中
        DJIMotorEnable(yaw_motor);
        DJIMotorSetRef(-2000);
        break;

    default:
        break;
    }


}