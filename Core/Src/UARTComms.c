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
#define PackageHead1 0xFA
#define PackageHead2 0xAF

uint8_t globalBuffer[50];
uint8_t receiveBuffer[50];
uint8_t receivedCMD;
uint8_t receivedData[48];

void UARTComms_Transmmit_Data(uint8_t cmd,uint8_t *data,uint8_t len)
{
	globalBuffer[0]=cmd;
	memcpy(&globalBuffer[1], data, len);
	CRC08_Append(globalBuffer, len + 2);
	memmove(globalBuffer+2, globalBuffer, len + 2);
	globalBuffer[0] = PackageHead1;
	globalBuffer[1] = PackageHead2;
	HAL_UART_Transmit_DMA(&huart2,globalBuffer,len+4);
}

void UARTComms_Recieve_Data(uint8_t *received,uint8_t len)
{
	if(received[0]==PackageHead1&&received[1]==PackageHead2)
	{
		memmove(received,received+2,len-2);
		len-=2;
		if(CRC08_Verify(received, len))
		{
			receivedCMD=received[0];
			memcpy(receivedData, received+1, len-1);
		}
		else{return;}
	}
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if(huart==&huart2)
	{
		UARTComms_Recieve_Data(receiveBuffer,Size);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, receiveBuffer, sizeof(receiveBuffer));
	}
}

void UARTComms_Init(void)
{
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, receiveBuffer, sizeof(receiveBuffer));
}
