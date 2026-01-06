#ifndef SMC_H
#define SMC_H
#include "main.h"
typedef struct
{
    float K;      // 滑模面系数
    float alpha;  // 指数趋近律参数
    float beta;   // 切换项增益
    float lambda; // 滑模面参数
    float dt;     // 控制周期
} SMC_Config;
typedef struct
{
    float actual_speed;   // 实际转速（RPM）
    float target_speed;   // 目标转速（RPM）
    float output;         // 输出电流
    float error;          // 速度误差
    float error_integral; // 误差积分
    SMC_Config config;    // 控制器配置
} SMC;

typedef struct {
    SMC controller;      // SMC控制器实例
    SMC_Config config;   // 配置参数
} DiffSMCController;
void SMC_Init(SMC_Config *config, float K, float alpha, float beta, float lambda, float dt, SMC *SMC_control);
float SMC_Calculate(SMC *SMC_control, SMC_Config *config, float set, float now);

#endif