#include "shoot.h"

static Publisher_t *shoot_pub;                  // 发射应用消息发布者(发射反馈给cmd)
static Subscriber_t *shoot_sub;                 // cmd控制消息订阅者
static Shoot_Upload_Data_s shoot_feedback_data; // 回传给cmd的发射状态信息
static Shoot_Ctrl_Cmd_s shoot_cmd_recv;         // 来自cmd的控制信息

static DJIMotorInstance *motor_l_steady, *motor_r_steady,*motor_release;

void Shoot_Init()
{
    Motor_Init_Config_s shoot_steady_motor_config = {
        .can_init_config.can_handle = &hcan1,
        .controller_param_init_config = {
            .current_PID = {
                .Kp = 0.5, // 4.5
                .Ki = 0,  // 0
                .Kd = 0,  // 0
                .IntegralLimit = 10000,
                .Improve = PID_Integral_Limit,
                .MaxOut = 20000,
            },

        },
        .controller_setting_init_config = {
            .force_feedback_source=OTHER_FEED,
            .outer_loop_type = CURRENT_LOOP,
            .close_loop_type = CURRENT_LOOP,
        },
        .motor_type = M3508,
    };
    Motor_Init_Config_s shoot_release_motor_config = {
        .can_init_config.can_handle = &hcan1,
        .controller_param_init_config = {
            .current_PID = {
                .Kp = 0.5, // 4.5
                .Ki = 0,  // 0
                .Kd = 0,  // 0
                .IntegralLimit = 10000,
                .Improve = PID_Integral_Limit,
                .MaxOut = 20000,
            },

        },
        .controller_setting_init_config = {
            .force_feedback_source=OTHER_FEED,
            .outer_loop_type = CURRENT_LOOP,
            .close_loop_type = CURRENT_LOOP,
        },
        .motor_type = M3508,
    };
    // 后两个稳速摩擦轮的注册与参数配置
    shoot_steady_motor_config.can_init_config.tx_id = 3; // 后左摩擦轮
    shoot_steady_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    shoot_steady_motor_config.controller_param_init_config.other_force_feedback_ptr = &sensor_info.current_left;
    motor_l_steady = DJIMotorInit(&shoot_steady_motor_config);

    shoot_steady_motor_config.can_init_config.tx_id = 4; // 后右摩擦轮
    shoot_steady_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    shoot_steady_motor_config.controller_param_init_config.other_force_feedback_ptr = &sensor_info.current_left;
    motor_r_steady = DJIMotorInit(&shoot_steady_motor_config);


    shoot_release_motor_config.can_init_config.tx_id = 1; // 后右摩擦轮
    shoot_release_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_release = DJIMotorInit(&shoot_release_motor_config);

    // SensorInit(&huart1);

    shoot_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));     // 回传给cmd的信息
    shoot_pub = PubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s)); // cmd传给发射的信息
}

void Shoot_Task()
{
        SensorSend();
       SubGetMessage(shoot_sub, &shoot_cmd_recv);

    if(shoot_cmd_recv.friction_mode == FRICTION_ON){
        DJIMotorEnable(motor_l_steady);
        DJIMotorSetRef(motor_l_steady,2433);
}
    else if (shoot_cmd_recv.friction_mode == FRICTION_OFF)
    {
        DJIMotorStop(motor_l_steady);
   
}
    PubPushMessage(shoot_pub, (void *)&shoot_feedback_data);
}