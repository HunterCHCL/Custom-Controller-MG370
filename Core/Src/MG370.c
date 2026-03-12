/*
 * MG370.c
 *
 *  Created on: Feb 28, 2026
 *      Author: HunterCHCL
 */
#include "MG370.h"
#include "cmsis_os2.h"

uint32_t MG370_A_EncoderCount = 0;
uint32_t MG370_B_EncoderCount = 0;

MG370_CascadePID_Motor_t MotorA_CascadeCtrl = {0};
MG370_CascadePID_Motor_t MotorB_CascadeCtrl = {0};


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
    uint16_t max_duty = (uint16_t)__HAL_TIM_GET_AUTORELOAD(&MG370_PWMA_TIMEBASE);
    if (duty > max_duty) {
        duty = max_duty;
    }
    __HAL_TIM_SET_COMPARE(&MG370_PWMA_TIMEBASE, MG370_PWMA, duty);
}

void MG370_B_SetPWM(uint16_t duty)
{
    uint16_t max_duty = (uint16_t)__HAL_TIM_GET_AUTORELOAD(&MG370_PWMB_TIMEBASE);
    if (duty > max_duty) {
        duty = max_duty;
    }
    __HAL_TIM_SET_COMPARE(&MG370_PWMB_TIMEBASE, MG370_PWMB, duty);
}

void MG370_A_ENCODER_Init(void)
{
    HAL_TIM_Encoder_Start(&MG370_A_Encoder, TIM_CHANNEL_ALL);
}

void MG370_B_ENCODER_Init(void)
{
    HAL_TIM_Encoder_Start(&MG370_B_Encoder, TIM_CHANNEL_ALL);
}


/**
 * @brief  获取并更新电机A的速度和角度
 * @param  motor 电机结构体
 */
void MG370_A_UpdateFeedback(MG370_CascadePID_Motor_t *motor)
{
    // 获取当前16位定时器的计数值
    uint16_t current_count = __HAL_TIM_GET_COUNTER(&MG370_A_Encoder);
    
    int16_t delta = (int16_t)(current_count - motor->last_encoder_count);
    delta = (int16_t)(delta * MG370_A_ENCODER_DIR);
    
    motor->current_speed = delta;
    motor->current_position += delta;
    motor->last_encoder_count = current_count;
}

/**
 * @brief  驱动电机A
 * @param  output_pwm  PWM输出
 */
void MG370_A_Drive(float output_pwm)
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
 * @brief  电机 A 的串级双环PID控制
 * @param  motor       电机A结构体
 * @param  target_pos  目标位置
 */
void MG370_A_CascadeControl(MG370_CascadePID_Motor_t *motor, int32_t target_pos)
{
    // 1. 设置外环期望目标位置
    motor->target_position = target_pos;
    
    // 2. 自发更新和计算电机的当前速度与无限界位置反馈
    MG370_A_UpdateFeedback(motor);
    
    // 3. 计算外环：“位置环”。把无边界的“距离差”换算为我们需要多快的“速度”去弥补
    float target_speed = PID_calc(&motor->position_pid, (float)motor->current_position, (float)motor->target_position);
    motor->target_speed = target_speed;
    
    // 4. 计算内环：“速度环”。把期望“速度”和“当前读取速度”间的误差化为最后实际要拉高的电压“PWM”值
    float pwm_out = PID_calc(&motor->speed_pid, (float)motor->current_speed, (float)motor->target_speed);
    motor->output_pwm = pwm_out;
    
    // 5. 应用到底层电机接口
    MG370_A_Drive(motor->output_pwm);
}


void MG370_B_UpdateFeedback(MG370_CascadePID_Motor_t *motor)
{
    uint16_t current_count = __HAL_TIM_GET_COUNTER(&MG370_B_Encoder);
    int16_t delta = (int16_t)(current_count - motor->last_encoder_count);
    delta = (int16_t)(delta * MG370_B_ENCODER_DIR);
    motor->current_speed = delta;
    motor->current_position += delta;
    motor->last_encoder_count = current_count;
}

void MG370_B_Drive(float output_pwm)
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

void MG370_B_CascadeControl(MG370_CascadePID_Motor_t *motor, int32_t target_pos)
{
    motor->target_position = target_pos;
    
    MG370_B_UpdateFeedback(motor);
    
    float target_speed = PID_calc(&motor->position_pid, (float)motor->current_position, (float)motor->target_position);
    motor->target_speed = target_speed;
    
    float pwm_out = PID_calc(&motor->speed_pid, (float)motor->current_speed, (float)motor->target_speed);
    motor->output_pwm = pwm_out;
    
    MG370_B_Drive(motor->output_pwm);
}

void StartMotorControll(void *argument)
{
    /* USER CODE BEGIN StartMotorControll */
    // 初始化电机底层外设
    MG370_Init();
    MG370_A_ENCODER_Init();
    MG370_B_ENCODER_Init();

    // 同步编码器初值
    MotorA_CascadeCtrl.last_encoder_count = (uint16_t)__HAL_TIM_GET_COUNTER(&MG370_A_Encoder);
    MotorB_CascadeCtrl.last_encoder_count = (uint16_t)__HAL_TIM_GET_COUNTER(&MG370_B_Encoder);

    float pwm_limit = (float)__HAL_TIM_GET_AUTORELOAD(&MG370_PWMA_TIMEBASE);
    
    // 电机A PID设置
    PID_init(&MotorA_CascadeCtrl.position_pid, PID_POSITION, (float[]){position_kp, position_ki, position_kd}, position_max_output, 0.0f);
    PID_init(&MotorA_CascadeCtrl.speed_pid, PID_POSITION, (float[]){speed_kp, speed_ki, speed_kd}, pwm_limit, speed_maxiout);
/** 
    MotorA_CascadeCtrl.position_pid.mode = PID_POSITION;
    MotorA_CascadeCtrl.speed_pid.mode = PID_POSITION;
    MotorA_CascadeCtrl.position_pid.Kp = position_kp;
    MotorA_CascadeCtrl.position_pid.Ki = position_ki;
    MotorA_CascadeCtrl.position_pid.Kd = position_kd;
    MotorA_CascadeCtrl.position_pid.max_out = position_max_output; // 最大输出速度限制
    MotorA_CascadeCtrl.position_pid.max_iout = 0.0f; // 位置环通常不需要积分项，避免稳态误差过大

    MotorA_CascadeCtrl.speed_pid.Kp = speed_kp;
    MotorA_CascadeCtrl.speed_pid.Ki = speed_ki;
    MotorA_CascadeCtrl.speed_pid.Kd = speed_kd;
    MotorA_CascadeCtrl.speed_pid.max_out = pwm_limit; // PWM 上限与 TIM3 ARR 对齐
    MotorA_CascadeCtrl.speed_pid.max_iout = speed_maxiout; // 积分限幅防止饱和过冲
*/
    // 电机 B PID 设置
    PID_init(&MotorB_CascadeCtrl.position_pid, PID_POSITION, (float[]){position_kp, position_ki, position_kd}, position_max_output, 0.0f);
    PID_init(&MotorB_CascadeCtrl.speed_pid, PID_POSITION, (float[]){speed_kp, speed_ki, speed_kd}, pwm_limit, speed_maxiout);
/**
    MotorB_CascadeCtrl.position_pid.mode = PID_POSITION;
    MotorB_CascadeCtrl.speed_pid.mode = PID_POSITION;
    MotorB_CascadeCtrl.position_pid.Kp = position_kp;
    MotorB_CascadeCtrl.position_pid.Ki = position_ki;
    MotorB_CascadeCtrl.position_pid.Kd = position_kd;
    MotorB_CascadeCtrl.position_pid.max_out = position_max_output; // 最大输出速度限制
    MotorB_CascadeCtrl.position_pid.max_iout = 0.0f; // 位置环通常不需要积分项，避免稳态误差过大
    
    MotorB_CascadeCtrl.speed_pid.Kp = speed_kp;
    MotorB_CascadeCtrl.speed_pid.Ki = speed_ki;
    MotorB_CascadeCtrl.speed_pid.Kd = speed_kd;
    MotorB_CascadeCtrl.speed_pid.max_out = pwm_limit; // PWM 上限与 TIM3 ARR 对齐
    MotorB_CascadeCtrl.speed_pid.max_iout = speed_maxiout; // 积分限幅防止饱和过冲
*/    
    uint32_t tick = osKernelGetTickCount();
    
    /* Infinite loop */
    while(1)
    {
        // 1. 获取最新编码器反馈并执行位置+速度反馈闭环控制
        MG370_A_CascadeControl(&MotorA_CascadeCtrl, MotorA_CascadeCtrl.target_position);
        MG370_B_CascadeControl(&MotorB_CascadeCtrl, MotorB_CascadeCtrl.target_position);

        // 2. 维持精确的周期执行 (10ms)
        tick += osKernelGetTickFreq() * MG370_CONTROL_PERIOD_MS / 1000;
        osDelayUntil(tick);
    }
    /* USER CODE END StartMotorControll */
}
