#ifndef __MNGSERIAL_H
#define __MNGSERIAL_H

#include "mySerial.h"
#include "main.h"
#include <stdio.h>
#include "recDecode.h"
#include "mySerial.h"
// #include "Config.h"

#define INFINITE_LOOP_START for(;;){osDelay(4);
#define INFINITE_LOOP_END }

extern myUartStruct commuUart;
struct transmit_package_struct
{
	uint8_t crc;						//0 crc校验码
	uint8_t lenth;						//1 总长度-2
	//以下部分使用crc校验
	uint8_t rec_index[5];					//2 接收包的编号
										//3+
	float debug_data[10];				// 32位*x
	uint8_t warn_cnt;					// 错误数量
	uint8_t error_state[3];				// 错误内容
} __attribute__((packed));	// 对齐设置,强制不进行补位操作
extern struct transmit_package_struct debugData_pkg;

/* 统计丢包情况 */
#define MAX_PACKETS 256			// 统计丢包率的总数

typedef struct {
	int packets[MAX_PACKETS]; // 包的接收状态
	int index;                // 当前索引
} PacketStats;

extern PacketStats stats_toReceiver; // 数据的丢包情况结构体
extern uint8_t GamepadLostConnection;

void initStats(PacketStats *stats);
void ArrangeSerialList();
void sendSuccess(PacketStats *stats, int n);
void sendFail(PacketStats *stats, int n);
uint8_t getLossRate(PacketStats *stats);


#endif

