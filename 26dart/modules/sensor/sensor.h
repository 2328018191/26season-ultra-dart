#ifndef __SENSOR_H
#define __SENSOR_H

#include "usart.h"
#include "referee_protocol.h"
#include "robot_def.h"
#include "bsp_usart.h"
#include "FreeRTOS.h"

typedef struct
{
 float weight_left;
 float weight_right;
 float weight;

 float current_left;
 float current_right;
 float current_sensor;
} sensor_info_t;
void SensorSend();
sensor_info_t *SensorInit(UART_HandleTypeDef *sensor_usart_handle);
void ParseBothWeight32(uint8_t *buff);

extern sensor_info_t sensor_info;			  // 传感器数据

#endif 