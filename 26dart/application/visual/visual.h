#ifndef VISUAL_H
#define VISUAL_H

#define MINIPC_SENDLENGTH 28U
#define MINIPC_REVCLENGTH 48U   //������Ҫ����Э���ֽ�
#define CV_SOF 0x5A
#define CV_EOF 0XA5
// #include "robot_task.h"    
#include "shoot.h"
#include "step_motor.h" 
#include "ins_task.h"
#include <stdbool.h>
#include "usbd_cdc_if.h"
#include "usb_device.h"
#include "crc_ref.h"

#pragma pack(1)

typedef struct
{
    uint8_t header;
    uint8_t detect_color : 1; // 0-red 1-blue
    uint8_t reset_tracker : 1;
    uint8_t reserved : 6;
    float roll;
    float pitch;
    float yaw;
    float aim_x;
    float aim_y;
    float aim_z;
    uint16_t checksum;

} CV_T_t;

typedef struct
{

        uint8_t header;
        uint8_t tracking : 1;
        uint8_t id : 3;         // 0-outpost 6-guard 7-base
        uint8_t armors_num : 3; // 2-balance 3-outpost 4-normal
        uint8_t reserved : 1;
        float x;
        float y;
        float z;
        float yaw;
        float vx;
        float vy;
        float vz;
        float v_yaw;
        float r1;
        float r2;
        float dz;
        uint16_t checksum;

    } CV_R_t;

  
  
  
    typedef  struct
    {
        uint8_t state; // ���״̬
        // /*
        // LOST				0x00
        // FIND_NO_SHOOT	0x01
        // FIND_SHOOT		0x02
        // FINDING			0x03
        // */
        float pitch; // pitch����Ƕ�
        float yaw;   // yaw����Ƕ�
        // 0����������1������
        uint8_t relive_flag;      // ��������ָ��
        uint8_t pill_flag;        // �����һ�����ָ��
        uint16_t pill_17mm_count; // ��Ҫ�һ���17mm�ӵ���
        uint8_t valid;
        float last_pitch;
        float last_yaw;
        float per_pitch;
        float per_yaw;
        uint8_t now_state;
        uint8_t last_state;
        uint8_t lost_flag;
        uint8_t check;
    
    } ext_gimbal_CV_ctrl_t;

    struct SolveTrajectoryParams
{
	float k; // ����ϵ��

	// ��������
	float current_v;              // ��ǰ����
	float current_pitch;          // ��ǰpitch
	float current_yaw;            // ��ǰyaw

	// Ŀ�����	
    int bias_time;            // ƫ��ʱ��
	float s_bias;             // ǹ��ǰ�Ƶľ���
	float z_bias;             // yaw������ǹ��ˮƽ�Ĵ�ֱ����

	// float xw;                 // ROS����ϵ�µ�x
	// float yw;                 // ROS����ϵ�µ�y
	// float zw;                 // ROS����ϵ�µ�z
	// float vxw;                // ROS����ϵ�µ�vx
	// float vyw;                // ROS����ϵ�µ�vy
	// float vzw;                // ROS����ϵ�µ�vz
	float tar_yaw;            // Ŀ��yaw
	float v_yaw;              // Ŀ��yaw�ٶ�
	// float r1;                 // Ŀ�����ĵ�ǰ��װ�װ�ľ���
	// float r2;                 // Ŀ�����ĵ�����װ�װ�ľ���
	// float dz;                 // ��һ��װ�װ������ڱ����ٵ�װ�װ�ĸ߶Ȳ�

	// enum BULLET_TYPE bullet_type; // �������������� 0-���� 1-Ӣ��
	// float aim_x;
	// float aim_y;
	// float aim_z;
};
extern Vision_Recv_s *vision;
extern ext_gimbal_CV_ctrl_t CV_EV;
extern struct SolveTrajectoryParams cv_st;
void Parse_data();
void NucToMcu(uint8_t *Buf, const uint32_t *Len);
void McuToNuc();


#endif