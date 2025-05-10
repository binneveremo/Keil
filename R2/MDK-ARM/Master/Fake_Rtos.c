#include "Communication.h"
#include "Television.h"
#include "Fake_Rtos.h"
#include "Location.h"
#include "Encoder.h"
#include "Correct.h"
#include "Chassis.h"
#include "Global.h"
#include "Flow.h"
#include "mine.h"
#include "Send.h"
#include "stdio.h"
#include "Gyro.h"
#include "RGB.h"

#include "CPU_Load.h"
#include "Interact.h"
#include "Basket.h"
void motor_control(void const * argument)
{
   for(;;)
  {
		switch(chassis.Control_Status){
			case gamepad_standard:
				GamePad_Velocity_Standard();    
			break;
			case gamepad_free_noheader:
				GamePad_Velocity_FreeNoheader();
			break;
			case gamepad_r1dir_noheader:
				GamePad_Velocity_R1DirNoheader();
			break;
			case progress:
				BasketPositionLock();
			break;
			case back:
				Back();
			break;
			case dribble:
				Dribbble_Flow();
			case safe:
				
			break;
		}
		//察觉到空置状态的变化
		ControlStatus_Detect();
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
		Vision_Basket_Decode();
		//手柄数据解析
		Get_GamePad_Data();
	  GamePad_Data_Cla();
	  Wireless_Send();
    osDelay(20);
	}
	
}
void location(void const * argument)
	
{
  for(;;)
  {
	  YIS506_Decode();
		//陀螺仪原始数据计算 
		Encoder_XY_VX_VY_Cal(2);
		//获取陀螺仪加速度
		Gyro_AX_AY_Cal();
		//编码器速度计与陀螺仪及速度计的融合
		Enc_VXVY_Fuse_With_Gyro_AXAY(2);
		//雷达与编码器的重定位融合
		//为车车选择坐标系
    Location_Type_Choose();
		//向视觉 发送定位以及速度
		Send_Velocity_Vision();
		//插帧得到篮筐和当前坐标的相关信息
		BasketPositionCal_AccordingVision(2);
		//DT35解算
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




























