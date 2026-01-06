#include "reload.h"
#include "DJi_motor.h"
#include "vofa.h"
#include "servo_motor.h"
#include "stdlib.h"
#include "string.h"
#include "Robot_cmd.h"
#include "cmsis_os.h"

// -------------------------- 原有全局变量保留 --------------------------
static Publisher_t *reload_pub;                   // 发射应用消息发布者(发射反馈给cmd)
static Subscriber_t *reload_sub;                  // cmd控制消息订阅者
static Reload_Upload_Data_s reload_feedback_data; // 回传给cmd的发射状态信息
static Reload_Ctrl_Cmd_s reload_cmd_recv;         // 来自cmd的控制信息

static DJIMotorInstance *reload_3508,*reload_6020;
static ServoInstance *servo_motor;
static ServoInstance *servo_motor_left;
static ServoInstance *servo_motor_right;
static ServoInstance *servo_motor_end;
static Reload_mode_e last_reload_mode;

static uint8_t servo_flag = 0;

//左129.5 中149-150 右166.5（你的原有目标角度）
const float TARGET_ANGLES[] = {149.5f,149.5f, 124.5f, 149.5f,166.5f,149.5f};
#define NUM_TARGET_ANGLES   (sizeof(TARGET_ANGLES) / sizeof(TARGET_ANGLES[0]))
static uint8_t current_target_index = 0;  // 当前目标角度的索引
static float now_reload_angle = 0;

static float init_angle = 0;
static float trigger_angle = 0;
static float Difference_angle = 0;
static float last_angle = 0;

static float reload_time = 0;
static float hibernate_time = 0;

// -------------------------- 新增状态机相关定义 --------------------------
// 换弹流程状态枚举
typedef enum {
    RELOAD_STATE_INIT,          // 初始化：夹取第一个物料
    RELOAD_STATE_IDLE_1,        // 空闲1：等待指令放第一个物料
    RELOAD_STATE_RELEASE_1,     // 执行：放下第一个物料
    RELOAD_STATE_WAIT_MOVE_2,   // 延时：放完后等待移动到第二个位置
    RELOAD_STATE_MOVE_2,        // 执行：移动到第二个位置
    RELOAD_STATE_IDLE_2,        // 空闲2：等待指令夹取第二个物料
    RELOAD_STATE_GRAB_2,        // 执行：夹取第二个物料
    RELOAD_STATE_RETURN_2,      // 执行：回位（放第二个物料的基础位置）
    RELOAD_STATE_WAIT_RELEASE_2,// 延时：回位后等待放第二个物料
    RELEASE_STATE_2,            // 执行：放下第二个物料
    RELOAD_STATE_WAIT_MOVE_3,   // 延时：放完后等待移动到第三个位置
    RELOAD_STATE_MOVE_3,        // 执行：移动到第三个位置
    RELOAD_STATE_IDLE_3,        // 空闲3：等待指令夹取第三个物料
    RELOAD_STATE_GRAB_3,        // 执行：夹取第三个物料
    RELOAD_STATE_RETURN_3,      // 执行：回位（放第三个物料的基础位置）
    RELOAD_STATE_WAIT_RELEASE_3,// 延时：回位后等待放第三个物料
    RELEASE_STATE_3,            // 执行：释放第三个物料
    RELOAD_STATE_FINISH         // 流程完成，重置为初始状态
} ReloadState;

// 全局状态变量
static ReloadState reload_state = RELOAD_STATE_INIT;
// 通用1秒延时（可根据需求调整）
static const float delay_1s = 1.0f;
// 记录每个阶段的开始时间戳
static float action_start_time = 0.0f;

// -------------------------- 原有初始化函数保留并优化 --------------------------
void Reload_Init()
{

    Motor_Init_Config_s reload_6020_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 1,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 250, // 8
                .Ki = 1,
                .Kd = 3,
                .MaxOut = 5000,
            },
            .speed_PID = {
                .Kp = 5, // 50
                .Ki = 0, // 200
                .Kd = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 3000,
                .MaxOut = 20000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = GM6020};

    reload_6020 = DJIMotorInit(&reload_6020_config);

    // 注册舵机
    Servo_Init_Config_s Servo_push_Init_Config =
        {
            .htim = &htim1,
            .Channel = TIM_CHANNEL_1,
        };
    servo_motor = ServoInit(&Servo_push_Init_Config);
    Servo_push_Init_Config.Channel = TIM_CHANNEL_2;
    servo_motor_left = ServoInit(&Servo_push_Init_Config);
    Servo_push_Init_Config.Channel = TIM_CHANNEL_3;
    servo_motor_right = ServoInit(&Servo_push_Init_Config);
    Servo_push_Init_Config.Channel = TIM_CHANNEL_4;
    servo_motor_end = ServoInit(&Servo_push_Init_Config);

    reload_pub = PubRegister("reload_feed", sizeof(Reload_Upload_Data_s));
    reload_sub = SubRegister("reload_cmd", sizeof(Reload_Ctrl_Cmd_s));
    
    // 初始上电归零到第一个目标角度
    DJIMotorEnable(reload_6020);
    DJIMotorOuterLoop(reload_6020, ANGLE_LOOP);
    DJIMotorSetRef(reload_6020, TARGET_ANGLES[0]);
    
    // 初始化夹取第一个物料（调用你的FirstPos函数）
    InitPos();
    
    // 初始化状态为等待放第一个物料
    reload_state = RELOAD_STATE_IDLE_1;
}

// -------------------------- 核心换弹任务函数（重写） --------------------------
void Reload_Task()
{
    // 1. 接收最新指令
    SubGetMessage(reload_sub, &reload_cmd_recv);
    // 2. 获取当前6020电机角度
    now_reload_angle = reload_6020->measure.total_angle;

    // 3. 状态机核心逻辑：按流程执行每个步骤
    switch (reload_state)
    {
        // ------------------- 第一阶段：初始化完成，等待放第一个物料 -------------------
        case RELOAD_STATE_IDLE_1:
            // 收到RELOAD_ONE_MODE指令：放下第一个物料
            if (reload_cmd_recv.reload_mode == RELOAD_ONE_MODE)
            {
                FirstPos();
                
                action_start_time = DWT_GetTimeline_s(); // 记录释放开始时间
                reload_state = RELOAD_STATE_WAIT_MOVE_2;
            }
            break;
              case RELOAD_STATE_WAIT_MOVE_2:
            // 延时1秒后，移动到第二个位置（TARGET_ANGLES[2] = 129.5f）
            if (DWT_GetTimeline_s() > action_start_time + 1.5f)
            {
                  Release();
                  action_start_time = DWT_GetTimeline_s(); // 记录释放开始时间
                  reload_cmd_recv.reload_mode = RELOAD_FREE_MODE; // 重置指令
                  reload_state = RELOAD_STATE_RELEASE_1;
            }
            break;

        case RELOAD_STATE_RELEASE_1:
            // 延时1秒后，移动到第二个位置（TARGET_ANGLES[2] = 129.5f）
            if (DWT_GetTimeline_s() > action_start_time + 1.5f)
            {
                DJIMotorSetRef(reload_6020, TARGET_ANGLES[2]); // 移到第二个位置
                action_start_time = DWT_GetTimeline_s(); // 记录释放开始时间

                reload_state = RELOAD_STATE_IDLE_2;
            }
            break;

        // ------------------- 第二阶段：等待夹取第二个物料 -------------------


        case RELOAD_STATE_IDLE_2:
            // 收到RELOAD_ONE_MODE指令：夹取第二个物料
            if (reload_cmd_recv.reload_mode == RELOAD_ONE_MODE)
            {
                Grab(); // 执行夹取第二个物料
                action_start_time = DWT_GetTimeline_s(); // 记录夹取开始时间
                reload_cmd_recv.reload_mode = RELOAD_FREE_MODE; // 重置指令
                reload_state = RELOAD_STATE_WAIT_MOVE_3;
            }
            break;
        case RELOAD_STATE_WAIT_MOVE_3:
            if (DWT_GetTimeline_s() > action_start_time + 1.5f){
                InitPos();
                reload_cmd_recv.reload_mode = RELOAD_FREE_MODE; // 重置指令
                action_start_time = DWT_GetTimeline_s(); // 记录夹取开始时间
                reload_state = RELOAD_STATE_GRAB_2;}
            break;


        case RELOAD_STATE_GRAB_2:
            // 夹取后回位到中间位置（TARGET_ANGLES[1] = 149.5f）
            if (DWT_GetTimeline_s() > action_start_time + 1.5f){

            DJIMotorSetRef(reload_6020, TARGET_ANGLES[1]);
            action_start_time = DWT_GetTimeline_s(); // 记录释放开始时间
            reload_state = RELEASE_STATE_2;}
            break;
        case RELEASE_STATE_2:
                if (DWT_GetTimeline_s() > action_start_time + 1.0f)
            {
                FirstPos(); // 放下第二个物料
                action_start_time = DWT_GetTimeline_s(); // 记录释放开始时间
                reload_state = RELOAD_STATE_RETURN_2;}
                break;
        case RELOAD_STATE_RETURN_2:
            // 回位后延时1秒，放下第二个物料
            if (DWT_GetTimeline_s() > action_start_time + 2.5f)
            {
                Release(); // 放下第二个物料
                action_start_time = DWT_GetTimeline_s(); // 记录释放开始时间
                reload_state = RELOAD_STATE_WAIT_RELEASE_2;
            }
            break;


        case RELOAD_STATE_WAIT_RELEASE_2:
            // 放完第二个物料后延时1秒，移动到第三个位置（TARGET_ANGLES[4] = 166.5f）
            if (DWT_GetTimeline_s() > action_start_time + 2.5f)
            {
                DJIMotorSetRef(reload_6020, TARGET_ANGLES[4]); // 移到第三个位置
                reload_state = RELOAD_STATE_IDLE_3;
            }
            break;

        // ------------------- 第三阶段：等待夹取第三个物料 -------------------
        // case RELOAD_STATE_WAIT_MOVE_3:
        //     // 等待电机到位
        //     if (fabs(now_reload_angle - TARGET_ANGLES[4]) < 1.0f)
        //     {
        //         reload_state = RELOAD_STATE_MOVE_3;
        //     }
        //     break;


        case RELOAD_STATE_IDLE_3:
            // 收到RELOAD_ONE_MODE指令：夹取第三个物料
            if (reload_cmd_recv.reload_mode == RELOAD_ONE_MODE)
            {
                Grab(); // 执行夹取第三个物料
                action_start_time = DWT_GetTimeline_s(); // 记录夹取开始时间
                reload_cmd_recv.reload_mode = RELOAD_FREE_MODE; // 重置指令
                reload_state = RELOAD_STATE_GRAB_3;
            }
            break;

        case RELOAD_STATE_GRAB_3:
            // 夹取后回位到中间位置（TARGET_ANGLES[5] = 149.5f）
            if (DWT_GetTimeline_s() > action_start_time + 1.5f){

            DJIMotorSetRef(reload_6020, TARGET_ANGLES[5]);
            action_start_time = DWT_GetTimeline_s(); // 记录夹取开始时间
            reload_state = RELOAD_STATE_RETURN_3;}
            break;

        case RELOAD_STATE_RETURN_3:
            // 回位后延时1秒，释放第三个物料
            if (DWT_GetTimeline_s() > action_start_time + 1.5f)
            {
                Release(); // 释放第三个物料并回到初始位置
                reload_state = RELOAD_STATE_FINISH;
            }
            break;

        // ------------------- 流程结束 -------------------
        case RELEASE_STATE_3:
        case RELOAD_STATE_FINISH:
            // 可选：重置为初始状态，循环执行；或保持空闲
            // reload_state = RELOAD_STATE_INIT;
            reload_state = RELOAD_STATE_FINISH; // 流程结束，不再响应指令
            break;

        // 异常处理
        default:
            reload_state = RELOAD_STATE_INIT; // 异常时重置为初始化
            FirstPos(); // 重新夹取第一个物料
            break;
    }
}