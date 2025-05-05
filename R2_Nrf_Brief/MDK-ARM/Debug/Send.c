#include "string.h"
#include "Send.h"
#include "Global.h"
#define send_uart huart1
#define wireless_uart huart10

#define RECEIVE_R1 1

#if  RECEIVE_R1
#define wireless_data_num 10
#else
#define wireless_data_num 16

#endif


int send_flag;
char send_data[40];
unsigned char wireless_data[wireless_data_num];
uint8_uint32_float_union send_union;

///////////////////////////无线烧录器所使用的程序////////////////////////////////////////////
uint8_uint32_float_union send_union;
void Send_Put_Data(char index,float data){
	send_union.float_data[index] = data;
}
void Send_Float_Data(char num){
	if(send_flag == 0)
		return;
	//计算前几位的值
	float total; 
	for(char i = 0; i < num;i++)
		total += send_union.float_data[i];
	//计算最后一位 也就是校验位的数值
	send_union.float_data[num] = total;
	//对发送的字节进行赋值
	memcpy(send_data,send_union.uint8_data,(num + 1) * 4);
	//发送
	HAL_UART_Transmit(&send_uart, (unsigned char*)send_data, (num + 1) * 4, HAL_MAX_DELAY);
}


/////////////////////////////////////////////无线网条所使用的程序////////////////////////////


void Wireless_Communication_Init(void){
	HAL_UART_Init(&wireless_uart);
	__HAL_UART_ENABLE_IT(&wireless_uart, UART_IT_IDLE);
	HAL_UARTEx_ReceiveToIdle_DMA(&wireless_uart, wireless_data, sizeof(wireless_data));
	__HAL_DMA_DISABLE_IT(wireless_uart.hdmarx, DMA_IT_HT);  // 禁用传输过半中断
}
void Wireless_Receive_Decode(UART_HandleTypeDef *huart){
	if(huart->Instance == wireless_uart.Instance){
		HAL_UARTEx_ReceiveToIdle_DMA(&wireless_uart, wireless_data, sizeof(wireless_data));
#if RECEIVE_R1
		Get_R1_Data(wireless_data);
#else
		Get_Vision_Data(wireless_data);
#endif
	}
}
void Wireless_Send(void){
	unsigned char send[10];
	send[0] = 0xAA;
	send_union.float_data[0] = site.now.x + 250;
	send_union.float_data[1] = site.now.y;
	memcpy(&send[1],send_union.uint8_data,8);
	send[9] = chassis.Flagof.self_lock;
	HAL_UART_Transmit(&wireless_uart, (unsigned char*)send, sizeof(send), HAL_MAX_DELAY);
 }
//板子测试程序
void Send_Test(void){
	unsigned char send1[4] = {0x01,0x02,0x03,0x04};
	HAL_UART_Transmit(&huart2, (unsigned char*)send1, sizeof(send1), HAL_MAX_DELAY);
	unsigned char send2[4] = {0x04,0x03,0x02,0x01};
	HAL_UART_Transmit(&huart6, (unsigned char*)send2, sizeof(send2), HAL_MAX_DELAY);
	osDelay(4);
}













