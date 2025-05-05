#ifndef __CHASSIS_H
#define __CHASSIS_H

#include "Communication.h"
#include "Location.h"
#include "Global.h"
#include "HO7213.h"
#include "mine.h"
#include "VESC.h"

#define Small_Green 0


#if Small_Green
//四个舵轮的编号
#define frontright 0
#define frontleft 1
#define behindleft 2
#define behindright 3
//转向轮 发送的ID
#define front_right_turn_send_id 1
#define front_left_turn_send_id 2
#define behind_left_turn_send_id 3
#define behind_right_turn_send_id 4
//转向轮 回传的ID
#define front_right_turn_receive_id 0x65
#define front_left_turn_receive_id 0x66
#define behind_left_turn_receive_id 0x67
#define behind_right_turn_receive_id 0x68
//驱动轮 VESC的ID
#define front_right_drive_id 6
#define front_left_drive_id 7 
#define behind_left_drive_id 8
#define behind_right_drive_id 9
//想要逆时针转 就把初始偏置调大 
//想要顺时针转 就把初始偏置调小 
#define frontright_offset -15
#define frontleft_offset 45
#define behindleft_offset  160
#define behindright_offset -63
#else
#define VESC_NUM 4
#define TURN_NUM 4
#define front_wheel 0		
#define left_wheel 1
#define right_wheel 2
#define behind_wheel 3
//转向轮 发送的ID
#define front_turn_send_id 4
#define left_turn_send_id 3
#define right_turn_send_id 2
#define behind_turn_send_id 1
//转向轮 回传的ID
#define front_turn_receive_id 4
#define left_turn_receive_id 3
#define right_turn_receive_id 2
#define behind_turn_receive_id 1
//驱动轮 VESC的ID
#define front_drive_id 9
#define left_drive_id 8
#define right_drive_id 7
#define behind_drive_id 6
//左转 偏置变大
#define front_offset 174
#define left_offset 174
#define right_offset  0
#define behind_offset  -156
#endif


struct Mark
{
	float p;
	float i;
	float istart;
	float iend;
	float ilimit;
	float outlimit;
	float gain;
	
	float itotal_x;
	float itotal_y;
	float outx;
	float outy;

	
	///////新加的测试参数
	float total_dis;
	float brake_ilimit;
	float brake_outlimit;
	float brake_distance;
	float brake_gain;
	float brake_percent;
};
struct Chassis{
	enum{
			safe,
			gamepad_standard,
			gamepad_free_noheader,
			gamepad_r1dir_noheader,
			flow,
			back,
	}Control_Status;
	struct {
		struct VESC  drive[VESC_NUM];
		struct HO7213 turn[TURN_NUM];
	}motor;
	struct{
		char self_lock;
	  char GamePad_Slow;
		char GamePad_Accel;
		char GamePad_Inverse;
	}Flagof;
};

extern struct Chassis chassis;
extern struct Mark mark;
//泡点参数初始化
void Velocity_Run_Par_Init(void);
//跑点
void Velocity_Run(float vx,float vy,float angle);
void Velocity_Run_Test(float vx,float vy,float vr);
//纠正角度
float Correct_Angle(float target);
void Correct_Angle_Par_Init(void);
//舵轮的输出以及解码
void Turn_Motor_Decode(int id,unsigned char * data);
void VectorWheel_SetSpeed(void);
void VectorWheel_SetAngle(void);
void Chassis_Velocity_Out(float left,float front,float anticlock);
//设置目标点
void Set_Target_Point(struct Point point);
//释放技能
void Release_Skill(int time);
//手柄遥控
void GamePad_Velocity_Standard(void);
void GamePad_Velocity_FreeNoheader(void);
void GamePad_Velocity_R1DirNoheader(void);
//到点判断
unsigned char Arrive_Point(struct Point point);
unsigned char Near_Point(struct Point point);
//自动自锁
void Self_Lock_Auto(void);
void Self_Brake_Out(void);
void Self_Lock_Out(void);
//涵道输出
void Set_Fun_Speed(void);
//s输入角度目标值 输出角度PID的输出值
float Correct_Angle(float target);

void Mark_PID_Par_Init(void);
void Position_With_Mark_PID_Run(void);

extern int Dribble_Begin;
void Dribble_Flow(void);
void Debug_Detect(void);


#endif



