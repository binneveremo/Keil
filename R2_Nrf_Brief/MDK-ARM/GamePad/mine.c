#include "Communication.h"
#include "Television.h"
#include "mngCommu.h"
#include "recDecode.h"
#include "Encoder.h"
#include "string.h"
#include "Global.h"
#include "string.h"
#include "Send.h"
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
char GamePad_Trigger(char flag,char index,int time){
	static int last[10];
	int now = HAL_GetTick();
	char trigger = ((flag == 1) && (now - last[index] > time))?1:0;
	//记录上一次触发的时间
	last[index] = (flag == 1)?HAL_GetTick():last[index];
	return trigger;
}

void GamePad_Data_Cla(void){
	//拨码状态切花
	if(Game_Pad_Switch_Data[5] == 1)
		chassis.Control_Status = safe;
	else if(Game_Pad_Switch_Data[3] == 1)
		chassis.Control_Status = gamepad_standard;
	else if(Game_Pad_Switch_Data[1] == 1)
		chassis.Control_Status = gamepad_free_noheader;
	else if(Game_Pad_Switch_Data[0] == 1)
		chassis.Control_Status = gamepad_r1dir_noheader;
	else if(Game_Pad_Switch_Data[8] == 1)
		chassis.Control_Status = progress;
	else if(Game_Pad_Switch_Data[7] == 1)
		chassis.Control_Status = dribble;
	else if(Game_Pad_Switch_Data[6] == 1)
		chassis.Control_Status = back;
	else 
		chassis.Control_Status = safe;
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
 		Tell_Yao_Xuan("dribble");
	if(Game_Pad_Key_Data[16] == 1)
 		Tell_Yao_Xuan("lift");
	if(Game_Pad_Key_Data[15] == 1)
 		Tell_Yao_Xuan("jump");
	//无线串口
	send_flag = Game_Pad_Switch_Data[1];
	//加速和减速相关
	chassis.Flagof.GamePad_Accel  = Game_Pad_Key_Data[21];
	chassis.Flagof.GamePad_Slow  = Game_Pad_Key_Data[8];
	//坐标系反转标志位
	chassis.Flagof.GamePad_Inverse = GamePad_Trigger(Game_Pad_Key_Data[7],0,200)?(!chassis.Flagof.GamePad_Inverse):chassis.Flagof.GamePad_Inverse;
	//vision.reset_flag = Game_Pad_Key_Data[1];
	vision.reset_flag = (GamePad_Trigger(Game_Pad_Key_Data[1],1,200) == 1)?1:vision.reset_flag;
	//清空码盘
	if(Game_Pad_Key_Data[5] == 1){
 		Odometer_Clear();
		Gyro_Reset();
	}
	//线程方面
	if(Game_Pad_Key_Data[3] == 1)
		Flow_Reset();
	//运球复位相关
	//切换debug模式
//	if(Game_Pad_Key_Data[19] == 1)
//	 debug = Change_Debug_Mode(debug);
} 
char * Show_Car_Status(void){
	return Car_Status;
}