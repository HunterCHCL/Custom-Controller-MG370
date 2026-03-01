/*
 * UARTComms.c
 *
 *  Created on: Feb 22, 2026
 *      Author: HunterCHCL
 */
#include "CRCs.h"
#include "UARTComms.h"
#include "usart.h"
#include "stdint.h"
#include "string.h"
#include "cmsis_os2.h"
#include "MG370.h"

uint8_t globalBuffer[50];
uint8_t receiveBuffer[50];
uint8_t receivedCMD;
uint8_t receivedData[48];

// 声明外部任务句柄，用于同步
extern osThreadId_t MotorControllHandle;
extern osThreadId_t CommsTaskHandle;

// 定义通讯标志位
#define COMMS_SIGNAL_RECEIVED 0x01

void UARTComms_Transmmit_Data(uint8_t cmd,uint8_t *data,uint8_t len)
{
	globalBuffer[0]=cmd;
	memcpy(&globalBuffer[1], data, len);
	CRC08_Append(globalBuffer, len + 2);
	memmove(globalBuffer+2, globalBuffer, len + 2);
	globalBuffer[0] = PackageHead1;
	globalBuffer[1] = PackageHead2;
	HAL_UART_Transmit_DMA(&UARTComms_Port,globalBuffer,len+4);
}

void UARTComms_Recieve_Data(uint8_t *received,uint8_t len)
{
    if(received[0]==PackageHead1&&received[1]==PackageHead2)
    {
        // 剥离包头 0xFA 0xAF (2字节)
        memmove(received,received+2,len-2);
        len-=2;
        // 此时 received[0] 是 cmd，received[1-4] 是数据，末尾是 CRC
        // CRC08_Verify 会根据内部算法校验 cmd+data 与最后的校验码
        if(CRC08_Verify(received, len))
        {
            receivedCMD=received[0];
            // 复制数据 (len-1) 字节 (排除 cmd 和 最后1字节 CRC)
            memcpy(receivedData, received+1, len-2);
        }
        else{return;}
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if(huart==&UARTComms_Port)
	{
		UARTComms_Recieve_Data(receiveBuffer,Size);
        
        // 发送任务通知（信号量），唤醒 CommsTask 处理数据
        // 注意：在中断中使用 osThreadFlagsSet 是安全的
        if (CommsTaskHandle != NULL) {
            osThreadFlagsSet(CommsTaskHandle, COMMS_SIGNAL_RECEIVED);
        }
        
		HAL_UARTEx_ReceiveToIdle_DMA(&UARTComms_Port, receiveBuffer, sizeof(receiveBuffer));
	}
}

void UARTComms_Init(void)
{
	HAL_UARTEx_ReceiveToIdle_DMA(&UARTComms_Port, receiveBuffer, sizeof(receiveBuffer));
}

void StartCommsTask(void *argument)
{
    UARTComms_Init();

    float angleA, angleB;
    int32_t countA, countB;
    uint32_t flags;

    while(1)
    {
        flags = osThreadFlagsWait(COMMS_SIGNAL_RECEIVED, osFlagsWaitAny, osWaitForever);

        if (flags & COMMS_SIGNAL_RECEIVED)
        {
            // 收到中断发来的处理信号，开始检查数据包内容
            if(receivedCMD == 0x01)
            {
                memcpy(&angleA, &receivedData[0], 4);
                memcpy(&angleB, &receivedData[4], 4);

                // 转换为编码器计数值
                countA = MG370_DEG_TO_COUNT(angleA);
                countB = MG370_DEG_TO_COUNT(angleB);

                // 更新电机控制结构体的目标位置
                MotorA_CascadeCtrl.target_position = countA;
                MotorB_CascadeCtrl.target_position = countB;

                // 处理完指令，将 global 指令标志归位
                receivedCMD = 0;
            }
        }
    }
}