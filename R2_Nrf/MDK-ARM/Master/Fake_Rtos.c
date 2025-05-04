#include "Communication.h"
#include "Television.h"
#include "Fake_Rtos.h"
#include "Location.h"
#include "Encoder.h"
#include "Correct.h"
#include "Chassis.h"
#include "Global.h"
#include "DT35.h"
#include "Flow.h"
#include "mine.h"
#include "Send.h"
#include "stdio.h"
#include "Gyro.h"
#include "RGB.h"

#include "CPU_Load.h"
#include "Interact.h"
struct Point p = {.x = 12985,.y = -4000,.r = -0.3};
void motor_control(void const * argument)
{
   for(;;)
  {
		switch(chassis.control_status){
			case Debug_Control:
				Flow_Test();
			break;
			case GamePad_Control:
				switch(chassis.gamepad_control){
					case standard:
				   GamePad_Velocity_Standard();                                                                                                                                                                                                                                                                                                                                            
					break;
					case noheader:
					 GamePad_Velocity_Noheader();
					break;
				}
			break;
			case Vision_Control:
				Television_Control_With_Guard();
			break;
			case Self_Control:
				Back_ToReset();
			break;
			case Safe_Control:
				
			break;
		}
		//Debug_Detect();`
		Self_Lock_Auto();
		VectorWheel_SetSpeed();
	  VectorWheel_SetAngle();
		osDelay(6);
	}
}
void communication(void const * argument)
{
  for(;;)
  {
		
		//手柄数据解析
		Get_GamePad_Data();
	  GamePad_Data_Cla();
		Vision_Data_Decode();
//		Send_Put_Data(0,ladar.rowx);
//		Send_Put_Data(1,ladar.rowy);
//		Send_Put_Data(2,ladar.rowr);
//		Send_Put_Data(3,12.3);
//		Send_Float_Data(4);
		Wireless_Send();
		//Send_Test();
		//发送串口数据
		DT35_Send_Pycharm();
    osDelay(40);
	}
	
}
int index,color = Purple,bright = 46,dt = 15;
void location(void const * argument)
	
{
  for(;;)
  {
		//YIS506_Fuse_With_Ladar_Angle(500);
		//陀螺仪解算
	  YIS506_Decode();
		//陀螺仪原始数据计算 
		Encoder_XY_VX_VY_Cal(2);
		//获取陀螺仪加速度
		Gyro_AX_AY_Cal();
		//编码器速度计与陀螺仪及速度计的融合
		Enc_VXVY_Fuse_With_Gyro_AXAY(2);
		//雷达与编码器的重定位融合
		//Ladar_With_Odometer_Kalman(2);
		//为车车选择坐标系
    Location_Type_Choose();
		//向视觉 发送定位以及速度
		//Send_Velocity_Vision();
		//DT35解算
		DT35_Cla_Flow();
		osDelay(2);
  }
}
void Detect(void const * argument)
{
  for(;;)
  {
		Can_Detect();
    osDelay(500);
  }
}
////////////////////////////////////////////////////////////璇老师的进程//////////////////////////////
void HTMotorControl(void const * argument)
{
	for(;;)
	{
		Single_Control();
		HighTorque_SendPosParam_f(&hfdcan1, 6);
		osDelay(4);
	}
}

void ParamsChange(void const * argument)
{
	for(;;)
	{
		Loop_Judgement();
		Overall_Control();
		osDelay(10);
	}
}




























