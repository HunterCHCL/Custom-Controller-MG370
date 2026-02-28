/*
 * MG370.c
 *
 *  Created on: Feb 28, 2026
 *      Author: HunterCHCL
 */
#include "MG370.h"

uint32_t MG370_A_EncoderCount = 0;
uint32_t MG370_B_EncoderCount = 0;

void MG370_Init(void)
{
    HAL_TIM_PWM_Start(&MG370_PWMA_TIMEBASE, MG370_PWMA);
    HAL_TIM_PWM_Start(&MG370_PWMB_TIMEBASE, MG370_PWMB);
}

void MG370_A_SetState(MG370_State state)
{
    switch(state)
    {
        case MG370_State_Forward:
            HAL_GPIO_WritePin(MG370_AIN1_GPIO_Port, MG370_AIN1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(MG370_AIN2_GPIO_Port, MG370_AIN2_Pin, GPIO_PIN_RESET);
            break;
        case MG370_State_Backward:
            HAL_GPIO_WritePin(MG370_AIN1_GPIO_Port, MG370_AIN1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(MG370_AIN2_GPIO_Port, MG370_AIN2_Pin, GPIO_PIN_SET);
            break;
        case MG370_State_Brake:
            HAL_GPIO_WritePin(MG370_AIN1_GPIO_Port, MG370_AIN1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(MG370_AIN2_GPIO_Port, MG370_AIN2_Pin, GPIO_PIN_SET);
            break;
        case MG370_State_Coast:
            HAL_GPIO_WritePin(MG370_AIN1_GPIO_Port, MG370_AIN1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(MG370_AIN2_GPIO_Port, MG370_AIN2_Pin, GPIO_PIN_RESET);
            break;
    }
}

void MG370_B_SetState(MG370_State state)
{
    switch(state)
    {
        case MG370_State_Forward:
            HAL_GPIO_WritePin(MG370_BIN1_GPIO_Port, MG370_BIN1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(MG370_BIN2_GPIO_Port, MG370_BIN2_Pin, GPIO_PIN_RESET);
            break;
        case MG370_State_Backward:
            HAL_GPIO_WritePin(MG370_BIN1_GPIO_Port, MG370_BIN1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(MG370_BIN2_GPIO_Port, MG370_BIN2_Pin, GPIO_PIN_SET);
            break;
        case MG370_State_Brake:
            HAL_GPIO_WritePin(MG370_BIN1_GPIO_Port, MG370_BIN1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(MG370_BIN2_GPIO_Port, MG370_BIN2_Pin, GPIO_PIN_SET);
            break;
        case MG370_State_Coast:
            HAL_GPIO_WritePin(MG370_BIN1_GPIO_Port, MG370_BIN1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(MG370_BIN2_GPIO_Port, MG370_BIN2_Pin, GPIO_PIN_RESET);
            break;
    }
}

void MG370_A_SetPWM(uint16_t duty)
{
    __HAL_TIM_SET_COMPARE(&MG370_PWMA_TIMEBASE, MG370_PWMA, duty);
}

void MG370_B_SetPWM(uint16_t duty)
{
    __HAL_TIM_SET_COMPARE(&MG370_PWMB_TIMEBASE, MG370_PWMB, duty);
}

void MG370_A_ENCODER_Init(void)
{
    HAL_TIM_Encoder_Start(&MG370_PWMA_TIMEBASE, TIM_CHANNEL_ALL);
}

void MG370_B_ENCODER_Init(void)
{
    HAL_TIM_Encoder_Start(&MG370_PWMB_TIMEBASE, TIM_CHANNEL_ALL);
}

void MG370_A_ENCODER_Update(void)
{
    MG370_A_EncoderCount = __HAL_TIM_GET_COUNTER(&MG370_A_Encoder);
}

void MG370_B_ENCODER_Update(void)
{
    MG370_B_EncoderCount = __HAL_TIM_GET_COUNTER(&MG370_B_Encoder);
}

/* ================== 下面为双环PID控制功能的具体实现 ================== */

MG370_CascadePID_Motor_t MotorA_CascadeCtrl = {0};
MG370_CascadePID_Motor_t MotorB_CascadeCtrl = {0};

/**
 * @brief  单级PID的核心统一运算函数
 * @param  pid      向此控制器结构体指针传入不同级环(串级/内环)的PID参数
 * @param  target   当前级的目标量
 * @param  measure  当前级的实际测量值
 * @retval 修改后计算得到的当前级PID输出值
 */
float MG370_PID_Calc(MG370_PID_Controller_t *pid, float target, float measure)
{
    float output;
    
    // 1. 计算当前误差
    pid->error = target - measure;
    
    // 2. 积分部分累加（需注意限幅或积分分离防止抗饱和风暴）
    pid->integral += pid->error;
    
    // 对积分项进行限幅
    if (pid->max_integral > 0.0f) {
        if (pid->integral > pid->max_integral) {
            pid->integral = pid->max_integral;
        } else if (pid->integral < -pid->max_integral) {
            pid->integral = -pid->max_integral;
        }
    }
    
    // 3. 计算PID总输出（由于周期固定，周期已经分别合并到 Ki 和 Kd 参数中）
    output = pid->kp * pid->error +
             pid->ki * pid->integral +
             pid->kd * (pid->error - pid->last_error);
             
    // 4. 存储上次误差供下次计算微分使用
    pid->last_error = pid->error;
    
    // 5. 对总输出项进行限幅
    if (pid->max_output > 0.0f) {
        if (output > pid->max_output) {
            output = pid->max_output;
        } else if (output < -pid->max_output) {
            output = -pid->max_output;
        }
    }
    
    return output;
}

/* --------------------------------- 电机 A 的实现模块 --------------------------------- */

/**
 * @brief  获取并更新电机A由于编码器增量而代表的速度和位置
 * @param  motor 指向需更新状态的电机A全局结构体
 */
void MG370_A_UpdateFeedback(MG370_CascadePID_Motor_t *motor)
{
    // 获取当前16位定时器的原始计数值
    uint16_t current_count = __HAL_TIM_GET_COUNTER(&MG370_A_Encoder);
    
    // 利用C语言无符号整型在运算时的溢出特性，再通过强转可以获取无跳变差值
    int16_t delta = (int16_t)(current_count - motor->last_encoder_count);
    
    // 周期内的距离增量即可当做速度
    motor->current_speed = delta;
    // 累加真正的无上界全局位置
    motor->current_position += delta;
    
    motor->last_encoder_count = current_count;
}

/**
 * @brief  将有符号的PWM数值转化成了A相H桥底层对应的正反转和PWM命令
 * @param  output_pwm  带有极性与占空比幅值的 PWM
 */
void MG370_A_Drive(int16_t output_pwm)
{
    if (output_pwm > 0)
    {
        MG370_A_SetState(MG370_State_Forward);
        MG370_A_SetPWM((uint16_t)output_pwm);
    }
    else if (output_pwm < 0)
    {
        MG370_A_SetState(MG370_State_Backward);
        MG370_A_SetPWM((uint16_t)(-output_pwm));
    }
    else
    {
        MG370_A_SetState(MG370_State_Brake);
        MG370_A_SetPWM(0);
    }
}

/**
 * @brief  电机 A 的串级双环PID控制入口，需要固定频率规律性调用
 * @param  motor       电机A绑定的状态管理主结构体
 * @param  target_pos  设定要求这只电机达到的期望位置
 */
void MG370_A_CascadeControl(MG370_CascadePID_Motor_t *motor, int32_t target_pos)
{
    // 1. 设置外环期望目标位置
    motor->target_position = target_pos;
    
    // 2. 自发更新和计算电机的当前速度与无限界位置反馈
    MG370_A_UpdateFeedback(motor);
    
    // 3. 计算外环：“位置环”。把无边界的“距离差”换算为我们需要多快的“速度”去弥补
    float target_speed = MG370_PID_Calc(&motor->position_pid,
                                        (float)motor->target_position, 
                                        (float)motor->current_position);
    motor->target_speed = (int16_t)target_speed;
    
    // 4. 计算内环：“速度环”。把期望“速度”和“当前读取速度”间的误差化为最后实际要拉高的电压“PWM”值
    float pwm_out = MG370_PID_Calc(&motor->speed_pid,
                                   (float)motor->target_speed, 
                                   (float)motor->current_speed);
    motor->output_pwm = (int16_t)pwm_out;
    
    // 5. 应用到底层电机接口
    MG370_A_Drive(motor->output_pwm);
}

/* --------------------------------- 电机 B 的实现模块 --------------------------------- */

/**
 * @brief  获取并更新电机B的速度和位置
 */
void MG370_B_UpdateFeedback(MG370_CascadePID_Motor_t *motor)
{
    uint16_t current_count = __HAL_TIM_GET_COUNTER(&MG370_B_Encoder);
    int16_t delta = (int16_t)(current_count - motor->last_encoder_count);
    motor->current_speed = delta;
    motor->current_position += delta;
    motor->last_encoder_count = current_count;
}

/**
 * @brief  底层带状态处理的B电机驱动
 */
void MG370_B_Drive(int16_t output_pwm)
{
    if (output_pwm > 0)
    {
        MG370_B_SetState(MG370_State_Forward);
        MG370_B_SetPWM((uint16_t)output_pwm);
    }
    else if (output_pwm < 0)
    {
        MG370_B_SetState(MG370_State_Backward);
        MG370_B_SetPWM((uint16_t)(-output_pwm));
    }
    else
    {
        MG370_B_SetState(MG370_State_Brake);
        MG370_B_SetPWM(0);
    }
}

/**
 * @brief  电机 B 的串级双环PID控制入口
 */
void MG370_B_CascadeControl(MG370_CascadePID_Motor_t *motor, int32_t target_pos)
{
    motor->target_position = target_pos;
    
    MG370_B_UpdateFeedback(motor);
    
    float target_speed = MG370_PID_Calc(&motor->position_pid,
                                        (float)motor->target_position, 
                                        (float)motor->current_position);
    motor->target_speed = (int16_t)target_speed;
    
    float pwm_out = MG370_PID_Calc(&motor->speed_pid,
                                   (float)motor->target_speed, 
                                   (float)motor->current_speed);
    motor->output_pwm = (int16_t)pwm_out;
    
    MG370_B_Drive(motor->output_pwm);
}

void StartMotorControll(void *argument)
{
    /* USER CODE BEGIN StartMotorControll */
    /* Infinite loop */
    for(;;)
    {
        // 这里可以放一些测试代码，或者在实际应用中根据需要调用 MG370_A_CascadeControl 和 MG370_B_CascadeControl
        osDelay(10); // 假设我们以 10ms 的周期调用控制函数
    }
    /* USER CODE END StartMotorControll */
}