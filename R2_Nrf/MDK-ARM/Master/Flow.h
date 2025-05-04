#ifndef __FLOW_H
#define __FLOW_H

#include "Send.h"

struct Car {
	struct {
		char received;
		char jumped;
		char defend_send;
		char jump_send;
		char back;
	}flag_of;
	enum{
		start,
		start2dunkpoint,
		wait_r1_ball,
		dunk,
		back,
		end,
	}state;
};
extern struct Point dunk_point;
extern struct Point basket_point;
extern struct Car car;

void Flow_Test1(void);
void Tell_Guo_Xuan_Jia(char *message);
void Car_State_Decode(int id,unsigned char * data);

void Velocity_Test(void);
void Debug_Flow(void);
void Flow_Test(void);
void Flow_Reset(void);

void Run_Point_Test(void);
//////////////////////雷达测试相关函数////////////////////////////////
void Ladar_Offset_Flow(void);
void Ladar_Offset_Test_Clear(void);
extern char ladar_offset_finish_flag;
///////////////////////编码器测试相关函数//////////////////////////////
void Odometer_Offset_Flow(void);
void Odometer_Offset_Test_Clear(void);
extern char Odometer_offset_finish_flag;
void Flow_Test2(void);
void Back_ToReset(void);
#endif



