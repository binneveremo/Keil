#ifndef __SEND_H
#define __SEND_H

#include "Television.h"
#include "Fake_Rtos.h"
#include <string.h>
#include "Global.h"
#include "usart.h"
#include "Basket.h"
#include <stdio.h>





void Send_Put_Data(char index,float data);
void Send_Float_Data(char num);

void Wireless_Receive_Decode(UART_HandleTypeDef *huart);
void Wireless_Send(void);
void Wireless_Communication_Init(void);
extern int send_flag;

typedef union __attribute__((packed))
{
	unsigned char uint8_data[40];
	float float_data[10];
} uint8_float_union;

//板子测试程序
void Send_Test(void);
#endif
