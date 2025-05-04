#include "Fake_Rtos.h"
#include "Communication.h"
#include "Television.h"
#include "mngSerial.h"
#include "recDecode.h"
#include "mySerial.h"
#include "Location.h"
#include <string.h>
#include <stdlib.h>
#include "usart.h"
#include "crc8.h"
#include "mine.h"

#include "DT35.h"

#define RecPkg_state	stats_toReceiver.packets[stats_toReceiver.index]
#define RecPkg_indexpp 	stats_toReceiver.index = (stats_toReceiver.index+1) % MAX_PACKETS;			

myUartStruct commuUart;

// 丢包统计结构体定义
PacketStats stats_toReceiver; // 接收机数据的丢包情况
uint32_t last_received_tick;
uint8_t GamepadLostConnection;

uint8_t pkg_period_cnt = 0;
#define pkg_period_cnt_inc	pkg_period_cnt=(pkg_period_cnt+1)%5;

struct transmit_package_struct debugData_pkg;

/* **********************测试代码,可删*************************** */
uint16_t fks[32],sws[32];
uint16_t fkscnt,swscnt;
int pkgcnt,pkgidx;
uint8_t pkgidx_last;
uint8_t pkg_per_sec;
uint8_t pkg_per_sec_cnt;
/* ************************************************************** */

// 定时任务,freertos执行
void StartTransmit_task() {
INFINITE_LOOP_START
	if(HAL_GetTick() - last_received_tick <= 200)
		GamepadLostConnection = 0;
	else 
		GamepadLostConnection = 1;
INFINITE_LOOP_END
}
// 接收任务,串口接收中断中调用
void ArrangeSerialList() {
#define commuRxbuff		commuUart.Uart_RxBuff[commuUart.Uart_RxBuffIndex]
#define commuRxCnt		commuUart.Uart_Rx_Cnt
	if(commuUart.Uart_RxFlag == 1) {
		if(commuRxCnt != get_lenth(commuRxbuff) + 2) return;	// 接收包位数检验
		if(!crc8_check(commuRxbuff+2, get_lenth(commuRxbuff), get_crc(commuRxbuff))) return;		// crc校验
		recDecodeData(commuRxbuff);
		debugData_pkg.rec_index[pkg_period_cnt] = get_pkgIndex();
		pkg_period_cnt_inc;

		// 判断断连
		last_received_tick = HAL_GetTick();
		GamepadLostConnection = 0;
		
		// 处理要发送的数据]
		memset(debugData_pkg.debug_data,0,40);
	  if(debug == 0){	
			debugData_pkg.debug_data[0] = site.now.x;
		  debugData_pkg.debug_data[1] = site.now.y;
			debugData_pkg.debug_data[2] = site.now.r;
			debugData_pkg.debug_data[4] = 8000 - dt35.ldt35.its_base_dis;
			debugData_pkg.debug_data[5] = dt35.rdt35.its_base_dis;
			debugData_pkg.debug_data[6] = dt35.bdt35.its_base_dis;
			debugData_pkg.debug_data[7] = 15000 - dt35.fdt35.its_base_dis;
			debugData_pkg.debug_data[9] = 1.11;
		}
		else if(debug == 1){
			debugData_pkg.debug_data[0] = ladar.x;
			debugData_pkg.debug_data[1] = ladar.y;
			debugData_pkg.debug_data[2] = ladar.r;

			debugData_pkg.debug_data[6] = dt35.fdt35.row_dis;
			debugData_pkg.debug_data[7] = dt35.fdt35.its_base_dis;
			debugData_pkg.debug_data[8] = dt35.fdt35.its_base_angle; 
			debugData_pkg.debug_data[9] = 2.22;
		}

/* ************************************************************** */
		
		if(pkg_period_cnt != 0) return;		// 每5个包返回一次ACK

		// 返回ack
		debugData_pkg.lenth = sizeof(debugData_pkg) - 2;
		debugData_pkg.crc = crc8_generate((const uint8_t*)&debugData_pkg+2, sizeof(debugData_pkg)-2);		// 生成crc校验码
		HAL_UART_Transmit_DMA(commuUart.huart, (uint8_t *)&debugData_pkg, sizeof(debugData_pkg));
		
		
	}
}


/* 丢包统计函数 */
void initStats(PacketStats *stats) {
	stats->index = 0;
	for(int j=0;j<MAX_PACKETS;j++) {
		stats->packets[j] = 0; // 初始化包状态
	}
}
void sendSuccess(PacketStats *stats, int n) {
	stats->packets[n] = 1; // 设置为发送成功标志
}
void sendFail(PacketStats *stats, int n) {
	stats->packets[n] = 0; // 设置为发送失败标志
}
uint8_t getLossRate(PacketStats *stats) {
	uint16_t count_s = 0;
	uint16_t count_f= 0;
	for (int i = 0; i < MAX_PACKETS; i++) {
			if (stats->packets[i] == 0) {
					count_f++;
			}
			if (stats->packets[i] == 1) {
					count_s++;
			}
	}
	return (double)count_f/(double)(count_s+count_f)*100.0f;
}
