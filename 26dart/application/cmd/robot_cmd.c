// app
#include "robot_def.h"
#include "Robot_cmd.h"
// module
#include "remote_control.h"
#include "message_center.h"
#include "dji_motor.h"
// bsp
#include "bsp_dwt.h"
#include "bsp_usart.h"
// #include "vofa.h"
// 私有宏,自动将编码器转换成角度值
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // 对齐时的角度,0-360
#define PTICH_HORIZON_ANGLE (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI) // pitch水平时电机的角度,0-360

/* cmd应用包含的模块实例指针和交互信息存储*/

static RC_ctrl_t *rc_data;  // 遥控器数据,初始化时返回


static Publisher_t *gimbal_cmd_pub;            // 云台控制消息发布者
static Subscriber_t *gimbal_feed_sub;          // 云台反馈信息订阅者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;      // 传递给云台的控制信息
static Gimbal_Upload_Data_s gimbal_fetch_data; // 从云台获取的反馈信息

static Publisher_t *reload_cmd_pub;            // 换弹控制消息发布者
static Subscriber_t *reload_feed_sub;          // 换弹反馈信息订阅者
static Reload_Ctrl_Cmd_s reload_cmd_send;      // 传递给换弹的控制信息
static Reload_Upload_Data_s reload_fetch_data; // 从换弹获取的反馈信息

static Publisher_t *shoot_cmd_pub;           // 发射控制消息发布者
static Subscriber_t *shoot_feed_sub;         // 发射反馈信息订阅者
static Shoot_Ctrl_Cmd_s shoot_cmd_send;      // 传递给发射的控制信息
static Shoot_Upload_Data_s shoot_fetch_data; // 从发射获取的反馈信息

static Robot_Status_e robot_state; // 机器人整体工作状态


 uint16_t last_switch_status;
 uint16_t now_switch_status;

static uint8_t last_trigger_status;



void RobotCMDInit()
{

    rc_data = RemoteControlInit(&huart3); // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
    // debug_data = DebuglInit(&huart1);

    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    shoot_cmd_pub = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    shoot_feed_sub = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
    reload_cmd_pub = PubRegister("reload_cmd", sizeof(Reload_Ctrl_Cmd_s));
    reload_feed_sub = SubRegister("reload_feed", sizeof(Reload_Upload_Data_s));

    robot_state = ROBOT_READY; // 启动时机器人进入工作模式,后续加入所有应用初始化完成之后再进入

}

const RC_ctrl_t* get_rc_point(void)
{
    return rc_data;
}

/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */



static void RemoteControlSet()
{
    
    // 这个代表的是上个拨杆状态
    now_switch_status = rc_data[TEMP].rc.switch_a;
    // 云台接收遥控器的具体参数
    gimbal_cmd_send.yaw = 0.4f * (float)rc_data[TEMP].rc.rocker_l_;
    // gimbal_cmd_send.pitch += 0.001f * (float)rc_data[TEMP].rc.rocker_l1;
    gimbal_cmd_send.pitch = rc_data[TEMP].rc.rocker_l1;

    // 换弹接收遥控器的具体参数
    reload_cmd_send.rev = 10.0f * (float)rc_data[TEMP].rc.rocker_r_;

    // 这个判断语句不能使用if elseif ，因为拨杆存在多种组合，if elseif是互斥的，只有一个真条件会被执行，组合拨杆将会失效
    //  右侧开关状态,所有模块无力状态，此时可以移动Pitch、Yaw并且自由换弹补弹
    if (switch_is_up(rc_data[TEMP].rc.switch_a)&&switch_is_up(rc_data[TEMP].rc.switch_b)&&switch_is_up(rc_data[TEMP].rc.switch_c)&&switch_is_up(rc_data[TEMP].rc.switch_d))
    {
        reload_cmd_send.reload_mode = RELOAD_FREE_MODE;
        gimbal_cmd_send.gimbal_mode = GIMABL_FREE_MODE;
        shoot_cmd_send.friction_mode = FRICTION_OFF;
    }
    // // 右侧开关状态[中],上步进左转其他固定
    // else if (switch_is_mid(rc_data[TEMP].rc.switch_right))
    // {
    //      reload_cmd_send.reload_mode = RELOAD_NO_MOVE;
    //     gimbal_cmd_send.gimbal_mode = GIMABL_PITCHLEFT;
    //     shoot_cmd_send.friction_mode = FRICTION_OFF;
    // }   
    // // 右侧开关状态[上]yaw步进左转其他固定
    // else if (switch_is_up(rc_data[TEMP].rc.switch_right))
    // {
    //     gimbal_cmd_send.gimbal_mode = GIMBAL_YAWLEFT;
    //      shoot_cmd_send.friction_mode = FRICTION_OFF;
    //     reload_cmd_send.reload_mode = RELOAD_NO_MOVE;
    // }






    // // 右侧开关[下]，左侧开关[中]，发射
    // if (switch_is_down(rc_data[TEMP].rc.switch_right) && switch_is_mid(rc_data[TEMP].rc.switch_left))
    // {
    //     reload_cmd_send.reload_mode = RELOAD_FREE_MODE;
    //     gimbal_cmd_send.gimbal_mode = GIMABL_FREE_MODE;
    //     shoot_cmd_send.friction_mode = FRICTION_ON;

    // }


    // // 发射
//     if (switch_is_N(now_switch_status))
//     {
//         reload_cmd_send.reload_mode = RELOAD_FREE_MODE;
//         gimbal_cmd_send.gimbal_mode = GIMABL_FREE_MODE;
//         shoot_cmd_send.friction_mode = FRICTION_ON;

//     }


//        if( switch_is_S(now_switch_status))
// {
//          shoot_cmd_send.friction_mode = FRICTION_OFF;


// }
//     // 右侧开关状态[上]&&左侧开关状态[上],上步进右转
//     if (switch_is_up(rc_data[TEMP].rc.switch_right)&&switch_is_up(rc_data[TEMP].rc.switch_left))
//     {
//      reload_cmd_send.reload_mode = RELOAD_NO_MOVE;
//       gimbal_cmd_send.gimbal_mode = GIMBAL_PITCH_RIGHT;
//        shoot_cmd_send.friction_mode = FRICTION_OFF;
//    }
//     //左上右中,YAW步进右转
//        if (switch_is_mid(rc_data[TEMP].rc.switch_right)&&switch_is_up(rc_data[TEMP].rc.switch_left))//上中
//     {
//      reload_cmd_send.reload_mode = RELOAD_FREE_MODE;
//       gimbal_cmd_send.gimbal_mode = GIMBAL_YAWRIGHT;
//        shoot_cmd_send.friction_mode = FRICTION_OFF;//此行留着同样为了测试，后续比赛时需要删除
//    }

    // 换弹控制,从最中间挡位拨到下方，换弹一次


     if(switch_is_up(now_switch_status) && switch_is_down(last_switch_status))
     {
         reload_cmd_send.reload_mode = RELOAD_ONE_MODE;
     }
    last_switch_status = rc_data[TEMP].rc.switch_a;



}

/**
 * @brief  紧急停止,包括遥控器左上侧拨轮打满/重要模块离线/双板通信失效等
 *         停止的阈值'300'待修改成合适的值,或改为开关控制.
 *
 * @todo   后续修改为遥控器离线则电机停止(关闭遥控器急停),通过给遥控器模块添加daemon实现
 *
 */
static void EmergencyHandler()
{
    // 拨轮的向下拨超过一半进入急停模式.注意向打时下拨轮是正
    // if (rc_data[TEMP].rc.dial > 300 || robot_state == ROBOT_STOP) // 还需添加重要应用和模块离线的判断
    // {
    //     robot_state = ROBOT_STOP;
    //     gimbal_cmd_send.gimbal_mode = GIMABL_FREE_MODE;
    //     reload_cmd_send.reload_mode = RELOAD_FREE_MODE;
    //     shoot_cmd_send.shoot_mode = SHOOT_OFF;
    //     shoot_cmd_send.friction_mode = FRICTION_OFF;
    // }
    // // 遥控器右侧开关为[上],恢复正常运行
    // if (switch_is_up(rc_data[TEMP].rc.switch_right))
    // {
    //     robot_state = ROBOT_READY;
    //     shoot_cmd_send.shoot_mode = SHOOT_ON;
    // }
}
/* void TestUSART(UART_HandleTypeDef *huart, float rev, float pitch) {
    char buffer[64]; // 足够大的缓冲区来存储字符串
    snprintf(buffer, sizeof(buffer), "rev: %f, Pitch: %f\r\n", rev, pitch);
    HAL_UART_Transmit(huart, (uint8_t *)buffer, strlen(buffer), 100);
} */
/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask()
{
    // 从其他应用获取回传数据

    
 
    SubGetMessage(shoot_feed_sub, &shoot_fetch_data);
    SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);
    SubGetMessage(reload_feed_sub, &reload_fetch_data);


   RemoteControlSet();
    // 其他应用所需的控制数据在remotecontrolsetmode和mousekeysetmode中完成设置
    PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
    PubPushMessage(reload_cmd_pub, (void *)&reload_cmd_send);
}
