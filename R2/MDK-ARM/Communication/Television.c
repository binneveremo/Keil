#include "Television.h"


struct Ladar ladar;
char guard_stop_flag;

unsigned char vision_data[64];
unsigned char guard_data[20];
unsigned char ladar_data[12];
unsigned char vision_send[24];
uint8_uint32_float_union vision_convert;


//#define ladar_offset 67.865f 
#define near 400


//获取视觉数据 
void Get_Vision_Data(unsigned char * data){
//	if((data[2] == 0x10) && (data[3] == 0x01))
	memcpy(ladar_data,data + sizeof(float),sizeof(ladar_data));
//	ladar.rowx = char2float(data + sizeof(float)) * 1000;
//	ladar.rowy = char2float(data + 2*sizeof(float)) * 1000;
//	ladar.rowr = rad2ang(char2float(data + 3*sizeof(float)));
}
void Ladar_Decode(void){
	memcpy(&vision_convert.uint8_data,ladar_data,12);
	ladar.rowx = vision_convert.float_data[0] * 1000;
	ladar.rowy = vision_convert.float_data[1] * 1000;
 	ladar.rowr = (vision_convert.float_data[2]) * 2;
	//计算偏置的坐标轴
	ladar.r = rad2ang(ladar.rowr);
	ladar.x = ladar.rowx - 196.39*(sin(2 * PI * 0.16 * ladar.rowr + 1.46) - sin( 1.49)) + 400;
	ladar.y = ladar.rowy - 182.20*(sin(2 * PI * 0.16 * ladar.rowr  -0.12) - sin(-0.12)) + 375;
}
struct Guard guard; 
void Duard_Decode(void){
	memcpy(&vision_convert.uint8_data,guard_data,16);
	
	guard.target.x = vision_convert.float_data[0] * 1000;
	guard.target.y = vision_convert.float_data[1] * 1000;
	guard.vx = vision_convert.float_data[2];
	guard.vy = vision_convert.float_data[3];
	
}
//整体解码
void Vision_Data_Decode(void){ 
	Ladar_Decode();
	//Duard_Decode();
	

}
//视觉控制时候的流程

//向视觉发送速度的函数
void Send_Velocity_Vision(void){
	memset(&vision_convert,0,sizeof(vision_convert));
	vision_convert.float_data[0] = site.ladar_pos.with_odo_x / 1000;
	vision_convert.float_data[1] = site.ladar_pos.with_odo_y / 1000;
	vision_convert.float_data[2] = site.gyro_pos.with_odo_vx;
	vision_convert.float_data[3] = site.gyro_pos.with_odo_vy;
	vision_convert.float_data[4] = ang2rad(site.now.r);

	memcpy(vision_send,vision_convert.uint8_data,20);
	FDCAN_Send(&hfdcan3,site_id,"STD",vision_send,"FD",20,"ON");
}
//向视觉发送时间戳的函数
void Send_Clock_Vision(void){
	vision_convert.uint32_data[0] = HAL_GetTick();
	memcpy(vision_send,vision_convert.uint8_data,4);
	FDCAN_Send(&hfdcan3,clock_id,"STD",vision_send,"FD",4,"ON");
}
//向视觉发送避障目标点的函数
void Television_Control_With_Guard(void){
	struct Point point;
	memcpy(&point,&guard.target,sizeof(point));
	Set_Target_Point(point);
	Guard_Trace_Run();
	if((fabs(guard.target.x) < 1) && (fabs(guard.target.y) < 1))
		Chassis_Velocity_Out(0,0,Correct_Angle(0));
}

//发送要移动的方向 要清空 20个字节只赋值了17个



