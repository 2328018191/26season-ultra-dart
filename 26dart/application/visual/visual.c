#include "visual.h"
#define center_x 0

attitude_t *ins_data;         // ��̬�������ݽṹ��
const float tolerance = 0.1f; // ������Χ
// ���ĵ�������Ϊ������ֵ����0,����Ϊ������ֵС��0
Vision_Recv_s *vision; // �Ӿ����ݽṹ��
// ���ݽ����Լ�������
uint8_t dat[MINIPC_SENDLENGTH];
uint8_t visual_rx_buffer[sizeof(CV_R_t)]; // �������ݻ�������
uint8_t visual_rx_buffer1[sizeof(CV_R_t)];
uint8_t visual_rx_buffer_last[sizeof(CV_R_t)]; // �������ݻ�������
uint8_t last_CV_EV_state = 0;
CV_R_t CV_R;
CV_T_t CV_T;
float t_r = 0.2f; // ����ʱ��
#define GRAVITY 9.78
#define PC_ABS(x) ((x > 0) ? (x) : (-x))
ext_gimbal_CV_ctrl_t CV_EV; // ����ṹ���������
struct SolveTrajectoryParams cv_st;

void NucToMcu(uint8_t *Buf, const uint32_t *Len)
{
  if (Verify_CRC16_Check_Sum(Buf, *Len) != true)
  {
    return;
  }

  if (Buf[0] == 0xA5)
  {

    /* store the receive data */

    memcpy(&CV_R, Buf, *Len);
  }
}

void McuToNuc()
{
  ins_data = INS_Init();
  // ͷ֡��ֵ
  CV_T.header = CV_SOF;
  // ŷ���Ǹ�ֵ
  CV_T.pitch = ins_data->Pitch/180.0f * PI;
  CV_T.yaw = ins_data->Yaw/180.0f * PI;
  CV_T.roll = ins_data->Roll/180.0f * PI;
  // CRCУ��
  CV_T.checksum = Get_CRC16_Check_Sum((uint8_t *)&CV_T, MINIPC_SENDLENGTH - 2, 0xffff);
  // Append_CRC16_Check_Sum(&CV_T, MINIPC_SENDLENGTH - 2, CV_SEND);
  memcpy(dat, (uint8_t *)&CV_T, MINIPC_SENDLENGTH);
  Parse_data();

  CDC_Transmit_FS(dat, MINIPC_SENDLENGTH);
}

/*
@brief �����������������ģ��
@param s:m ����
@param v:m/s �ٶ�
@param angle:rad �Ƕ�
@return z:m
*/
float monoDirectionalAirResistanceModel(float s, float v, float angle)
{
  float z;
  // tΪ����v��angleʱ�ķ���ʱ��
  t_r = (float)((exp(cv_st.k * s) - 1) / (cv_st.k * v * cos(angle)));
  // zΪ����v��angleʱ�ĸ߶�
  z = (float)(v * sin(angle) * t_r - GRAVITY * t_r * t_r / 2.0f);
  // printf("model %f %f\n", t, z);
  return z;
}

/*
@brief pitch�����
@param s:m ����
@param z:m �߶�
@param v:m/s
@return angle_pitch:rad
*/

float pitchTrajectoryCompensation(float s, float z, float v)
{
  float z_temp, z_actual, dz;
  float angle_pitch;
  int i = 0;
  z_temp = z;
  if (s < 0)
  {
    return 0;
  }
  // iteration
  for (i = 0; i < 20; i++)
  {
    angle_pitch = atan2(z_temp, s); // rad
    z_actual = monoDirectionalAirResistanceModel(s, v, angle_pitch);
    dz = 0.3f * (z - z_actual);
    z_temp = z_temp + dz;
    // printf("iteration num %d: angle_pitch %f, temp target z:%f, err of z:%f, s:%f\n",
    //     i + 1, angle_pitch * 180 / PI, z_temp, dz,s);
    if (fabsf(dz) < 0.00001f)
    {
      break;
    }
  }
  return angle_pitch;
}


float debug =0;
float t = 0;
float t_u = 0;
static void autoSolveTrajectory(float *pitch, float *yaw, float *aim_x, float *aim_y, float *aim_z)
{

  // // ����Ԥ��
  // float timeDelay = cv_st.bias_time / 1000.0 + t_r;
  // cv_st.tar_yaw += cv_st.v_yaw * timeDelay;

  // ע��pitch��yaw�ķ���
  *pitch = -pitchTrajectoryCompensation(sqrt((*aim_x) * (*aim_x) + (*aim_y) * (*aim_y)),
                                        *aim_z + cv_st.z_bias, cv_st.current_v);
  *yaw = (float)(atan2(*aim_y, *aim_x));
  t=*aim_x;
  t_u= *aim_y;
}



void Parse_data()
{
  ins_data = INS_Init();
  float aim_x = 0.0f, aim_y = 0.0f, aim_z = 0.0f;
  float pitch = 0.0f;
  float yaw = 0.0f;
  cv_st.current_v = 27.5f;
  cv_st.current_pitch = ins_data->Pitch/180*PI; // ��ǰpitch
  cv_st.current_yaw = ins_data->Yaw/180*PI;     // ��ǰyaw

  aim_x = CV_R.x;
  aim_y = CV_R.y;
  aim_z = CV_R.z;

  // ��ȡ����ʱ����bias_time

  // Ԥ��ʱ����Ҫ���Ͽ���ʱ�䣬Ϊ1/������/1000.0f


  CV_EV.pitch = -(cv_st.current_pitch - pitch); // ����������Ŀ��pitch-��ǰpitch
  CV_EV.yaw = -(cv_st.current_yaw-yaw);  // ��ֵת��Ϊ�������ת���Ƕ�,�������ת���Ƕ�����˿�˵��̶�Ӧ
}

// // �����Ӿ��ش�����,������ת��Ϊfloat����
// float x = vision->x;
// // float y = vision_data->y;
// // float z = vision_data->z;

// if (fabsf(x - center_x) < tolerance)
// {
//     // x ������Ŀ�������ڣ�ֹͣ�������
//     high_speed = 44500;
//     low_speed = 42000;
//     StepMotorStop();
//     LOGINFO("X coordinate is within the target range. Stopping motor.");
// }
// else if (x > center_x)
// {
//     // x �������Ŀ��ֵ�������������ת
//     high_speed = 44500;
//     low_speed = 42000;
//     Step_Motor_Control_right();
//     LOGINFO("X coordinate is greater than the target. Moving motor to the right.");
// }
// else
// {
//     // x ����С��Ŀ��ֵ�������������ת
//     high_speed = 38800;
//     low_speed = 32000;
//     Step_Motor_Control_left();
//     LOGINFO("X coordinate is less than the target. Moving motor to the left.");
// }
