/*
 * UARTComms.h
 *
 *  Created on: Feb 22, 2026
 *      Author: HunterCHCL
 */

#ifndef INC_UARTCOMMS_H_
#define INC_UARTCOMMS_H_

#define PackageHead1 0xFA
#define PackageHead2 0xAF

#define UARTComms_Port huart2

extern uint8_t receivedData[48];
extern uint8_t receivedCMD;
void UARTComms_Transmmit_Data(uint8_t cmd,uint8_t *data,uint8_t len);
void UARTComms_Init(void);

#endif /* INC_UARTCOMMS_H_ */
