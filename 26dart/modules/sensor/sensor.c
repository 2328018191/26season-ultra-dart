/**
 * @file Sensor.C
 * @author kidneygood (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-11-18
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "sensor.h"
#include "string.h"
#include "crc_ref.h"
#include "bsp_usart.h"
#include "task.h"
#include "daemon.h"
#include "bsp_log.h"
#include "cmsis_os.h"

#define RE_RX_BUFFER_SIZE 255u // 传感器接收缓冲区大小

static USARTInstance *sensor_usart_instance; // 传感器串口实例
static DaemonInstance *sensor_daemon;		  // 传感器守护进程
sensor_info_t sensor_info ={0};			  // 传感器数据

// /**
//  * @brief 打包0x02设置灯光颜色指令
//  * @param team: 目标阵营（TRIGGER_TEAM_RED / TRIGGER_TEAM_BLUE）
//  * @param send_buff: 发送缓冲区（至少5字节）
//  * @retval 发送数据长度（单位：字节）
//  */
// uint16_t DartTrigger2_PackSetColorCmd(TriggerTeamType team, uint8_t *send_buff)
// {
//     if (send_buff == NULL) return 0;

//     // 帧格式：帧头(1) + 命令码(1) + 数据长度(1) + 数据内容(1) + 校验位(1)
//     send_buff[0] = 0XAA;  // 帧头：0xAA
//     send_buff[1] = 0X02; // 命令码：0x02
//     send_buff[2] = 0x01;                  // 数据长度：1字节（仅阵营参数）
//     send_buff[3] = 0X00;         // 数据内容：阵营值红方0x00；蓝方0x01
//     send_buff[4] = Get_CRC8_Check_Sum(send_buff, 4, 0X00); // CRC8校验

//     return 5; // 0x02命令固定帧长5字节
// }


int32_t w1,w2;
 void ParseBothWeight32(uint8_t *buff)
{
	if (buff == NULL)	   // 空数据包，则不作任何处理
		return;
    if (buff[0] != 0x01 || buff[1] != 0x03) return;

     w1  = ((int32_t)((uint16_t)buff[5] | ((uint16_t)buff[6] << 8)) << 16)
           |  (uint16_t)buff[3] | ((uint16_t)buff[4] << 8);
     w2  = ((int32_t)((uint16_t)buff[9] | ((uint16_t)buff[10] << 8)) << 16)
           |  (uint16_t)buff[7] | ((uint16_t)buff[8] << 8);      
    sensor_info.weight_left = (float)abs(w1) / 25600.0f;
    sensor_info.weight_right =(float)abs(w2) / 25600.0f;
    sensor_info.current_left=sensor_info.weight_left/0.641*4000;
    sensor_info.current_right=sensor_info.weight_right/0.641*4000;

}

/*传感器串口接收回调函数,解析数据 */
static void SensorRxCallback()
{
	DaemonReload(sensor_daemon);
	ParseBothWeight32(sensor_usart_instance->recv_buff);
}
// 传感器丢失回调函数,重新初始化传感器串口
static void SensorLostCallback(void *arg)
{
	USARTServiceInit(sensor_usart_instance);
	LOGWARNING("[rm_ref] lost Sensor data");
}

/* 传感器通信初始化 */
sensor_info_t *SensorInit(UART_HandleTypeDef *sensor_usart_handle)
{
	USART_Init_Config_s conf;
	conf.module_callback = SensorRxCallback;
	conf.usart_handle = sensor_usart_handle;
	conf.recv_buff_size = RE_RX_BUFFER_SIZE; // mx 255(u8)
	sensor_usart_instance = USARTRegister(&conf);

	Daemon_Init_Config_s daemon_conf = {
		.callback = SensorLostCallback,
		.owner_id = sensor_usart_instance,
		.reload_count = 30, // 0.3s没有收到数据,则认为丢失,重启串口接收
	};
	sensor_daemon = DaemonRegister(&daemon_conf);

	return &sensor_info;
}

/**
 * @brief 传感器数据发送函数
 * @param
 */
void SensorSend()
{
    uint8_t uart_TxBuffer_2[8] = {0x01,0x03,0x00,0x00,0x00,0x04,0x44,0x09};

    // 发送指令
	  HAL_UART_Transmit(&huart1, uart_TxBuffer_2, 8, 100); // 发送数据
}
