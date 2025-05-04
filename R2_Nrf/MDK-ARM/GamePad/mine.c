#include "Communication.h"
#include "Television.h"
#include "mngCommu.h"
#include "recDecode.h"
#include "Encoder.h"
#include "string.h"
#include "Global.h"
#include "string.h"
#include "Send.h"
#include "DT35.h"
#include "RGB.h"
#include "Flow.h"
#include "Gyro.h"
#include "mine.h"
char debug;
char long_no_connect;

char Car_Status[5];
char Control_Status[5];

float Game_Pad_Rocker_Data[4];
unsigned char Game_Pad_Key_Data[22];
unsigned char Game_Pad_Switch_Data[10];
void GamePad_Init(void){
	Commu_init();
}
void Get_GamePad_Data(void){
	for(int i = 0; i< 4;i++)
	 Game_Pad_Rocker_Data[i] = get_rocker(i);
	for(int i = 0; i< 22;i++)
	 Game_Pad_Key_Data[i] = get_fk_state(i);
	for(int i = 0; i< 10;i++)
	 Game_Pad_Switch_Data[i] = get_sw_state(i);
}
char Change_Debug_Mode(char flag){
	static int last;
	int now = HAL_GetTick();
	if(now - last >= 230){
		last = now;
		return !flag;
	}
	return flag;
}
void GamePad_Detect(char time){
	static char cnt;
	if(GamepadLostConnection == 1)
		cnt++;
	if(cnt >= time){
		cnt = 0;
		GamePad_Init();
	}
}
///////////////////////////////////手柄按键内部定时器 也就是在flag出发的时候就会返回信号 但是在time时间以内再次出发就不会返回触发信号
char GamePad_Timer(char flag,char index,int time){
	static int last[10];
	int now = HAL_GetTick();
	char trigger = ((flag == 1) && (now - last[index] > time))?1:0;
	//记录上一次触发的时间
	last[index] = (flag == 1)?HAL_GetTick():last[index];
	return trigger;
}

void GamePad_Data_Cla(void){
	//底盘状态切换
	/*
	2:手柄
	3.无头
	4.视觉
	
	6 Debug
	7 技能
	8.自控
	9.none
	*/
	if(Game_Pad_Switch_Data[5] == 1)
		chassis.control_status = Safe_Control;
	else if(Game_Pad_Switch_Data[3] == 1){
		chassis.control_status = GamePad_Control;
		chassis.gamepad_control = standard;
	}
	else if(Game_Pad_Switch_Data[1] == 1){
		chassis.control_status = GamePad_Control;
		chassis.gamepad_control = noheader;
	}
	else if(Game_Pad_Switch_Data[8] == 1)
		chassis.control_status = Debug_Control;
	else if(Game_Pad_Switch_Data[6] == 1)
		chassis.control_status = Self_Control;
	else
		chassis.control_status = Safe_Control;

	//通信方面
	if(Game_Pad_Key_Data[4] == 1)
 		Tell_Yao_Xuan("fold");
	if(Game_Pad_Key_Data[6] == 1)
 		Tell_Yao_Xuan("catch");
	if(Game_Pad_Key_Data[2] == 1)
 		Tell_Yao_Xuan("predunk");
	if(Game_Pad_Key_Data[0] == 1)
 		Tell_Yao_Xuan("defend");
	
	
	if(Game_Pad_Key_Data[18] == 1)
 		Tell_Yao_Xuan("down");
	if(Game_Pad_Key_Data[17] == 1)
 		Tell_Yao_Xuan("dibble");
	if(Game_Pad_Key_Data[16] == 1)
 		Tell_Yao_Xuan("lift");
	if(Game_Pad_Key_Data[15] == 1)
 		Tell_Yao_Xuan("jump");
	//无线串口
	send_flag = Game_Pad_Switch_Data[1];
	DT35_Send_Flag = Game_Pad_Key_Data[17];
	Dribble_Velocity = Game_Pad_Key_Data[16]? Dribble_Velocity + 20:Dribble_Velocity;
	//加速和减速相关
	GamePad_Velocity_Standard_AccelFlag  = Game_Pad_Key_Data[21];
	//
	GamePad_Velocity_Standard_InverseFlag = GamePad_Timer(Game_Pad_Key_Data[8],0,200)?(!GamePad_Velocity_Standard_InverseFlag):GamePad_Velocity_Standard_InverseFlag;
	//清空码盘
	if(Game_Pad_Key_Data[5] == 1){
 		Odometer_Clear();
		Gyro_Reset();
	}
	//线程方面
	if(Game_Pad_Key_Data[3] == 1)
		Flow_Reset();
	//运球复位相关
	Dribble_Velocity = (Game_Pad_Key_Data[8] == 1)?9400:Dribble_Velocity;
	Dribble_Begin    = (Game_Pad_Key_Data[8] == 1)?0:Dribble_Begin;
	
	//切换debug模式
	if(Game_Pad_Key_Data[19] == 1)
	 debug = Change_Debug_Mode(debug);
	//切换锁头方向
	GamePad_NoHeader_R1Dir_Flag = Game_Pad_Switch_Data[0];
} 
char * Show_Car_Status(void){
	return Car_Status;
}