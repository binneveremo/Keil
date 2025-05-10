#include "mngCommu.h"
#include <string.h>
#include <stdlib.h>
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "recDecode.h"
#include "mine.h"
#include "crc8.h"

// other
#include "Location.h"


#ifdef USE_UDP
// 定义UDP目标IP
uint8_t des_ip[5] = DES_IP_DEF;
#endif

#ifdef USE_UART
myUartStruct commuUart;
#endif

#ifdef USE_NRF
Nrf_t nrf;
#endif

// 丢包统计结构体定义
uint32_t last_received_tick;
uint8_t GamepadLostConnection;

uint32_t timediff;

uint8_t pkg_period_cnt = 0;
#define pkg_period_cnt_inc	pkg_period_cnt=(pkg_period_cnt+1)%5;

struct transmit_package_struct debugData_pkg;

/* ************************************************************** */

// 定时任务,freertos执行
void StartTransmit_task(void const * argument) {
	static uint8_t cnt;
	osDelay(200);
INFINITE_LOOP_START
	
#ifdef USE_NRF
	// 处理接收数据
	if(HAL_GPIO_ReadPin(nrf.irq.port, nrf.irq.pin) == GPIO_PIN_RESET)
	{
		Nrf_EXTI_Callback(&nrf, nrf.irq.pin);
	}
#endif

	// 返回ack
	debugData_pkg.lenth = sizeof(debugData_pkg) - 2;
	debugData_pkg.crc = crc8_generate((const uint8_t*)&debugData_pkg+2, sizeof(debugData_pkg)-2);		// 生成crc校验码

	if(++cnt == 80)
	{cnt = 0;
#ifdef USE_UART
	HAL_UART_Transmit_DMA(commuUart.huart, (uint8_t *)&debugData_pkg, sizeof(debugData_pkg));
#endif
#ifdef USE_UDP
	myUdp_send((uint8_t *)&debugData_pkg, sizeof(debugData_pkg), des_ip[0], des_ip[1], des_ip[2], des_ip[3], DES_PORT);
#endif
#ifdef USE_NRF
	_Nrf_CheckConnectivity(&nrf);
	Nrf_Transmit(&nrf, (uint8_t *)&debugData_pkg, sizeof(debugData_pkg));
#endif
	
		// 清除已回报的包
		debugData_pkg.rec_cnt = 0;
	}
	
	// 判断断连
	if(HAL_GetTick() - last_received_tick > 350)
	{
		GamepadLostConnection = 1;
		Nrf_Reset(&nrf);
	}
INFINITE_LOOP_END
}

void Commu_init(void)
{
#ifdef USE_UART
	Serial_init(&Uartcomu, &TX_Uart);
#endif
#ifdef USE_UDP
	myUdp_init(MY_PORT);
#endif
#ifdef USE_NRF
	nrf = NRF_INIT;
	Nrf_Init(&nrf);
#endif
}

void Mng_RxData(uint8_t *pdata, uint16_t data_length, uint8_t reply_mode)
{
	if(data_length != get_lenth(pdata) + 2) return;	// 接收包位数检验
//	if(!crc8_check(pdata+2, get_lenth(pdata), get_crc(pdata))) return;		// crc校验
	recDecodeData(pdata);
	debugData_pkg.rec_cnt += 1;
	pkg_period_cnt_inc;

	// 判断断连
	last_received_tick = HAL_GetTick();
	GamepadLostConnection = 0;
	
// 处理要发送的数据
		if (debug == 0)
		{
			debugData_pkg.debug_data[0] = site.now.x;
			debugData_pkg.debug_data[1] = site.now.y;
			debugData_pkg.debug_data[2] = site.now.r;
			

		}
		else if (debug == 1)
		{
			
		}
//		Errorcode_Load();
/* ************************************************************** */
	
	if(pkg_period_cnt != 0) return;		// 每5个包返回一次ACK

	// 返回ack
	debugData_pkg.lenth = sizeof(debugData_pkg) - 2;
	debugData_pkg.crc = crc8_generate((const uint8_t*)&debugData_pkg+2, sizeof(debugData_pkg)-2);		// 生成crc校验码

#ifdef USE_UART
	HAL_UART_Transmit_DMA(commuUart.huart, (uint8_t *)&debugData_pkg, sizeof(debugData_pkg));
#endif
#ifdef USE_UDP
	myUdp_send((uint8_t *)&debugData_pkg, sizeof(debugData_pkg), des_ip[0], des_ip[1], des_ip[2], des_ip[3], DES_PORT);
#endif
#ifdef USE_NRF
	Nrf_Transmit(&nrf, (uint8_t *)&debugData_pkg, sizeof(debugData_pkg));
#endif
}


#ifdef USE_UART
// 接收任务,串口接收中断中调用
void ArrangeSerialList(void) {
#define commuRxbuff		commuUart.Uart_RxBuff[commuUart.Uart_RxBuffIndex]
#define commuRxCnt		commuUart.Uart_Rx_Cnt
	if(commuUart.Uart_RxFlag == 1) {
		Mng_RxData(commuRxbuff, commuRxCnt, 1);
	}
}
#endif

#ifdef USE_UDP
void ArrangeUdpData(void)
{
#define RxBuff_udp	myUdpStruct.RxBuff
#define RxCnt_udp	Uartcomu.Uart_Rx_Cnt
	Mng_RxData(RxBuff_udp, RxCnt_udp, 2);
}
#endif

#ifdef USE_NRF
void NrfCommu_EXTI_Callback(uint8_t channel, uint8_t *data, uint8_t len)
{
	Mng_RxData(data, len, 3);
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
//	Nrf_EXTI_Callback(&nrf, GPIO_Pin);  
}
#endif


