/*
 * MG370.h
 *
 *  Created on: Feb 28, 2026
 *      Author: HunterCHCL
 */

#ifndef INC_MG370_H_
#define INC_MG370_H_

#include "main.h"
#include "tim.h"

#define MG370_Transmission_Ratio 30
#define MG370_Encoder_Resolution 13
#define MG370_PWMA TIM_CHANNEL_1
#define MG370_PWMB TIM_CHANNEL_2
#define MG370_PWMA_TIMEBASE htim3
#define MG370_PWMB_TIMEBASE htim3
#define MG370_A_Encoder htim1
#define MG370_B_Encoder htim2

#define MG370_AIN1_Pin GPIO_PIN_12
#define MG370_AIN1_GPIO_Port GPIOB
#define MG370_AIN2_Pin GPIO_PIN_13
#define MG370_AIN2_GPIO_Port GPIOB
#define MG370_BIN1_Pin GPIO_PIN_14
#define MG370_BIN1_GPIO_Port GPIOB
#define MG370_BIN2_Pin GPIO_PIN_15
#define MG370_BIN2_GPIO_Port GPIOB

#define MG370_E1A_Pin GPIO_PIN_8
#define MG370_E1A_GPIO_Port GPIOA
#define MG370_E1B_Pin GPIO_PIN_9
#define MG370_E1B_GPIO_Port GPIOA
#define MG370_E2A_Pin GPIO_PIN_10
#define MG370_E2A_GPIO_Port GPIOA
#define MG370_E2B_Pin GPIO_PIN_11
#define MG370_E2B_GPIO_Port GPIOA

typedef enum
{
    MG370_State_Forward = 0,
    MG370_State_Backward = 1,
    MG370_State_Brake = 2,
    MG370_State_Coast = 3
} MG370_State;

void MG370_Init(void);
void MG370_A_SetState(MG370_State state);
void MG370_B_SetState(MG370_State state);
void MG370_A_ENCODER_Init(void);
void MG370_B_ENCODER_Init(void);

/* ================== PID 及双环控制相关结构体 ================== */

/** 
 *  PID 控制器结构体
 */
typedef struct {
    float kp;             // 比例系数
    float ki;             // 积分系数
    float kd;             // 微分系数
    
    float error;          // 当前偏差
    float last_error;     // 上一次偏差
    float integral;       // 积分累计值
    
    float max_integral;   // 积分限幅
    float max_output;     // 输出限幅
} MG370_PID_Controller_t;

/**
 *  单只电机的双环控制状态与参数结构体
 */
typedef struct {
    MG370_PID_Controller_t position_pid; // 位置外环PID（通过位置差计算目标速度）
    MG370_PID_Controller_t speed_pid;    // 速度内环PID（通过速度差计算PWM）
    
    int32_t target_position;      // 目标位置
    int32_t current_position;     // 当前累计无界位置
    
    int16_t current_speed;        // 当前速度 (即每一个更新周期的编码器增量)
    uint16_t last_encoder_count;  // 上一次记录的16位编码器原始计数值
    
    int16_t target_speed;         // 目标速度 (外环PID输出结果)
    int16_t output_pwm;           // 输出PWM占空比 (内环PID输出结果)
} MG370_CascadePID_Motor_t;

extern MG370_CascadePID_Motor_t MotorA_CascadeCtrl;
extern MG370_CascadePID_Motor_t MotorB_CascadeCtrl;

/* ================== PID 双环控制函数声明 ================== */

// PID 基础数学计算
float MG370_PID_Calc(MG370_PID_Controller_t *pid, float target, float measure);

// 电机状态的更新与提取（计算速度、累计位置）
void MG370_A_UpdateFeedback(MG370_CascadePID_Motor_t *motor);
void MG370_B_UpdateFeedback(MG370_CascadePID_Motor_t *motor);

// 处理带符号的PWM指令来控制正反转
void MG370_A_Drive(int16_t output_pwm);
void MG370_B_Drive(int16_t output_pwm);

// 核心的双环级联控制主函数（建议放在定时器中以恒定频率调用，例如 10ms）
void MG370_A_CascadeControl(MG370_CascadePID_Motor_t *motor, int32_t target_pos);
void MG370_B_CascadeControl(MG370_CascadePID_Motor_t *motor, int32_t target_pos);

#endif /* INC_MG370_H_ */
